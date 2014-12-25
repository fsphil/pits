/*------------------------------------------------------------------\
|                                                                   |
|                PI IN THE SKY TRACKER PROGRAM                      |
|                                                                   |
| This program is written for the Pi Telemetry Board                |
| produced by HAB Supplies Ltd.  No support is provided             |
| for use on other hardware. It does the following:                 |
|                                                                   |
| 1 - Sets up the hardware including putting the GPS in flight mode |
| 2 - Reads the current temperature                                 |
| 3 - Reads the current battery voltage                             |
| 4 - Reads the current GPS position                                |
| 5 - Builds a telemetry sentence to transmit                       |
| 6 - Sends it to the MTX2/NTX2B radio transmitteR                  |
| 7 - repeats steps 2-6                                             |
|                                                                   |
| 12/10/14 - Modifications for PITS+ V0.7 board and B+              |
| 11/11/14 - Modifications for PITS+ V2.0 board and A+/B+           |
| 19/12/14 - New GPS code.  Frequency calcs.  Image filenames       |
| 20/12/14 - Added APRS code.  ** Not yet flight tested **          |
|                                                                   |
\------------------------------------------------------------------*/

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdint.h>
#include <stdlib.h>
#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <wiringPi.h>
#include "gps.h"
#include "DS18B20.h"
#include "adc.h"
#include "misc.h"
#include "snapper.h"
#include "led.h"
#include "bmp085.h"
#include "aprs.h"
#include "lora.h"

struct TConfig Config;

// Pin allocations.  Do not change unless you're using your own hardware
#define NTX2B_ENABLE	0
#define UBLOX_ENABLE	2

FILE *ImageFP;
int Records;
struct termios options;
char *SSDVFolder="/home/pi/pits/tracker/download";
 
void BuildSentence(char *TxLine, int SentenceCounter, struct TGPS *GPS)
{
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	char TimeBuffer1[12], TimeBuffer2[10], ExtraFields1[20], ExtraFields2[20];
	
	sprintf(TimeBuffer1, "%06ld", GPS->Time);
	TimeBuffer2[0] = TimeBuffer1[0];
	TimeBuffer2[1] = TimeBuffer1[1];
	TimeBuffer2[2] = ':';
	TimeBuffer2[3] = TimeBuffer1[2];
	TimeBuffer2[4] = TimeBuffer1[3];
	TimeBuffer2[5] = ':';
	TimeBuffer2[6] = TimeBuffer1[4];
	TimeBuffer2[7] = TimeBuffer1[5];
	TimeBuffer2[8] = '\0';
	
	ExtraFields1[0] = '\0';
	ExtraFields2[0] = '\0';
	
	if (NewBoard())
	{
		sprintf(ExtraFields1, ",%u", GPS->BoardCurrent);
	}
	
	if (Config.EnableBMP085)
	{
		sprintf(ExtraFields2, ",%.1f,%.0f", GPS->ExternalTemperature, GPS->Pressure);
	}
	
    sprintf(TxLine, "$$%s,%d,%s,%7.5lf,%7.5lf,%05.5u,%d,%d,%d,%3.1f,%3.1f%s",
            Config.Channels[RTTY_CHANNEL].PayloadID,
            SentenceCounter,
			TimeBuffer2,
            GPS->Latitude,
            GPS->Longitude,
            GPS->Altitude,
			(GPS->Speed * 13) / 7,
			GPS->Direction,
			GPS->Satellites,            
            GPS->InternalTemperature,
            GPS->BatteryVoltage,
			ExtraFields1,
			ExtraFields2);

    Count = strlen(TxLine);

    CRC = 0xffff;           // Seed
    xPolynomial = 0x1021;
   
     for (i = 2; i < Count; i++)
     {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
     }

    TxLine[Count++] = '*';
    TxLine[Count++] = Hex((CRC >> 12) & 15);
    TxLine[Count++] = Hex((CRC >> 8) & 15);
    TxLine[Count++] = Hex((CRC >> 4) & 15);
    TxLine[Count++] = Hex(CRC & 15);
	TxLine[Count++] = '\n';  
	TxLine[Count++] = '\0';  

    printf("RTTY: %s", TxLine);
}


void ReadString(FILE *fp, char *keyword, char *Result, int Length, int NeedValue)
{
	char line[100], *token, *value;
 
	fseek(fp, 0, SEEK_SET);
	*Result = '\0';

	while (fgets(line, sizeof(line), fp) != NULL)
	{
		line[strcspn(line, "\r")] = '\0';			// Ignore any CR (in case someone has edited the file from Windows with notepad)
		
		token = strtok(line, "=");
		if (strcasecmp(keyword, token) == 0)
		{
			value = strtok(NULL, "\n");
			strcpy(Result, value);
			return;
		}
	}

	if (NeedValue)
	{
		printf("Missing value for '%s' in configuration file\n", keyword);
		exit(1);
	}
}

int ReadInteger(FILE *fp, char *keyword, int NeedValue, int DefaultValue)
{
	char Temp[64];

	ReadString(fp, keyword, Temp, sizeof(Temp), NeedValue);

	if (Temp[0])
	{
		return atoi(Temp);
	}
	
	return DefaultValue;
}

int ReadBoolean(FILE *fp, char *keyword, int NeedValue, int *Result)
{
	char Temp[32];

	ReadString(fp, keyword, Temp, sizeof(Temp), NeedValue);

	if (*Temp)
	{
		*Result = (*Temp == '1') || (*Temp == 'Y') || (*Temp == 'y') || (*Temp == 't') || (*Temp == 'T');
	}
	else
	{
		*Result = 0;
	}
	
	return *Temp;
}

int ReadBooleanFromString(FILE *fp, char *keyword, char *searchword)
{
	char Temp[100];

	ReadString(fp, keyword, Temp, sizeof(Temp), 0);

	if (strcasestr(Temp, searchword)) return 1; else return 0;
}

speed_t BaudToSpeed(int baud)
{
	switch (baud)
	{
		case 50: return B50;
		case 75: return B75;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
	}

	return 0;
}

void LoadConfigFile(struct TConfig *Config)
{
	const char *LoRaModes[5] = {"slow", "SSDV", "repeater", "turbo", "TurboX"};
	FILE *fp;
	int BaudRate, Channel;
	char Keyword[64];
	char *filename = "/boot/pisky.txt";

	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf("\nFailed to open config file %s (error %d - %s).\nPlease check that it exists and has read permission.\n", filename, errno, strerror(errno));
		exit(1);
	}

	ReadString(fp, "payload", Config->Channels[RTTY_CHANNEL].PayloadID, sizeof(Config->Channels[RTTY_CHANNEL].PayloadID), 1);
	printf ("RTTY Payload ID = '%s'\n", Config->Channels[RTTY_CHANNEL].PayloadID);

	ReadString(fp, "frequency", Config->Frequency, sizeof(Config->Frequency), 0);

	ReadBoolean(fp, "disable_monitor", 0, &(Config->DisableMonitor));
	if (Config->DisableMonitor)
	{
		printf("HDMI/Composite outputs will be disabled\n");
	}
	
	Config->EnableGPSLogging = ReadBooleanFromString(fp, "logging", "GPS");
	if (Config->EnableGPSLogging) printf("GPS Logging enabled\n");

	Config->EnableTelemetryLogging = ReadBooleanFromString(fp, "logging", "Telemetry");
	if (Config->EnableTelemetryLogging) printf("Telemetry Logging enabled\n");
	
	ReadBoolean(fp, "enable_bmp085", 0, &(Config->EnableBMP085));
	if (Config->EnableBMP085)
	{
		printf("BMP085 Enabled\n");
	}

	BaudRate = ReadInteger(fp, "baud", 1, 300);
	Config->TxSpeed = BaudToSpeed(BaudRate);
	if (Config->TxSpeed == B0)
	{
		printf ("Unknown baud rate %d\nPlease edit in configuration file\n", BaudRate);
		exit(1);
	}
	printf ("Radio baud rate = %d\n", BaudRate);

	ReadBoolean(fp, "camera", 0, &(Config->Camera));
	printf ("Camera %s\n", Config->Camera ? "Enabled" : "Disabled");
	if (Config->Camera)
	{
		Config->high = ReadInteger(fp, "high", 0, 0);
		printf ("Image size changes at %dm\n", Config->high);
	
		Config->low_width = ReadInteger(fp, "low_width", 0, 0);
		Config->low_height = ReadInteger(fp, "low_height", 0, 0);
		printf ("Low image size %d x %d pixels\n", Config->low_width, Config->low_height);
	
		Config->high_width = ReadInteger(fp, "high_width", 0, 0);
		Config->high_height = ReadInteger(fp, "high_height", 0, 0);
		printf ("High image size %d x %d pixels\n", Config->high_width, Config->high_height);

		Config->image_packets = ReadInteger(fp, "image_packets", 0, 0);
		printf ("1 Telemetry packet every %d image packets\n", Config->image_packets);
	}
	
	// I2C overrides.  Only needed for users own boards, or for some of our prototypes
	if (ReadInteger(fp, "SDA", 0, 0))
	{
		Config->SDA = ReadInteger(fp, "SDA", 0, 0);
		printf ("I2C SDA overridden to %d\n", Config->SDA);
	}

	if (ReadInteger(fp, "SCL", 0, 0))
	{
		Config->SCL = ReadInteger(fp, "SCL", 0, 0);
		printf ("I2C SCL overridden to %d\n", Config->SCL);
	}
	
	// APRS settings
	ReadString(fp, "APRS_Callsign", Config->APRS_Callsign, sizeof(Config->APRS_Callsign), 0);
	Config->APRS_ID = ReadInteger(fp, "APRS_ID", 0, 0);
	Config->APRS_Period = ReadInteger(fp, "APRS_Period", 0, 0);
	if (*(Config->APRS_Callsign) && Config->APRS_ID && Config->APRS_Period)
	{
		printf("APRS enabled for callsign %s:%d every %d minute%s\n", Config->APRS_Callsign, Config->APRS_ID, Config->APRS_Period, Config->APRS_Period > 1 ? "s" : "");
	}
	
	// LORA
	if (NewBoard())
	{
		// For dual card
		Config->LoRaDevices[0].DIO0 = 31;
		Config->LoRaDevices[0].DIO5 = 26;

		Config->LoRaDevices[1].DIO0 = 6;
		Config->LoRaDevices[1].DIO5 = 5;
	}
	else
	{
		Config->LoRaDevices[0].DIO0 = 6;
		Config->LoRaDevices[0].DIO5 = 5;
		
		Config->LoRaDevices[0].InUse = 1;
		Config->LoRaDevices[1].InUse = 0;
	}
	/*
	Config->LoRaDevices[0].DIO0 =  6;
	Config->LoRaDevices[0].DIO5 =  5;
	Config->LoRaDevices[0].InUse = 0;

	Config->LoRaDevices[1].DIO0 =  3;
	Config->LoRaDevices[1].DIO5 =  2;
	Config->LoRaDevices[1].InUse = 0;
	*/

	for (Channel=0; Channel<=1; Channel++)
	{
		int Temp;
		char TempString[64];
		
		strcpy(Config->LoRaDevices[Channel].LastCommand, "None");
		
		Config->LoRaDevices[Channel].Frequency[0] = '\0';
		sprintf(Keyword, "LORA_Frequency_%d", Channel);
		ReadString(fp, Keyword, Config->LoRaDevices[Channel].Frequency, sizeof(Config->LoRaDevices[Channel].Frequency), 0);
		if (Config->LoRaDevices[Channel].Frequency[0])
		{
			printf("LoRa Channel %d frequency set to %s\n", Channel, Config->LoRaDevices[Channel].Frequency);
			Config->LoRaDevices[Channel].InUse = 1;

			sprintf(Keyword, "LORA_Payload_%d", Channel);
			ReadString(fp, Keyword, Config->Channels[LORA_CHANNEL+Channel].PayloadID, sizeof(Config->Channels[LORA_CHANNEL+Channel].PayloadID), 1);
			printf ("LORA Channel %d Payload ID = '%s'\n", Channel, Config->Channels[LORA_CHANNEL+Channel].PayloadID);
			
			sprintf(Keyword, "LORA_Mode_%d", Channel);
			Config->LoRaDevices[Channel].SpeedMode = ReadInteger(fp, Keyword, 0, 0);
			printf("LoRa Channel %d %s mode\n", Channel, LoRaModes[Config->LoRaDevices[Channel].SpeedMode]);
			
			sprintf(Keyword, "LORA_Cycle_%d", Channel);
			Config->LoRaDevices[Channel].CycleTime = ReadInteger(fp, Keyword, 0, 0);			
			if (Config->LoRaDevices[Channel].CycleTime > 0)
			{
				printf("LoRa Channel %d cycle time %d\n", Channel, Config->LoRaDevices[Channel].CycleTime);

				sprintf(Keyword, "LORA_Slot_%d", Channel);
				Config->LoRaDevices[Channel].Slot = ReadInteger(fp, Keyword, 0, 0);
				printf("LoRa Channel %d Slot %d\n", Channel, Config->LoRaDevices[Channel].Slot);

				sprintf(Keyword, "LORA_Repeat_%d", Channel);
				Config->LoRaDevices[Channel].RepeatSlot = ReadInteger(fp, Keyword, 0, 0);			
				printf("LoRa Channel %d Repeat Slot %d\n", Channel, Config->LoRaDevices[Channel].RepeatSlot);

				sprintf(Keyword, "LORA_Uplink_%d", Channel);
				Config->LoRaDevices[Channel].UplinkSlot = ReadInteger(fp, Keyword, 0, 0);			
				printf("LoRa Channel %d Uplink Slot %d\n", Channel, Config->LoRaDevices[Channel].UplinkSlot);

				sprintf(Keyword, "LORA_Binary_%d", Channel);
				ReadBoolean(fp, Keyword, 0, &(Config->LoRaDevices[Channel].Binary));			
				printf("LoRa Channel %d Set To %s\n", Channel, Config->LoRaDevices[Channel].Binary ? "Binary" : "ASCII");
			}

			if (Config->LoRaDevices[Channel].SpeedMode == 4)
			{
				// Testing
				Config->LoRaDevices[Channel].ImplicitOrExplicit = IMPLICIT_MODE;
				Config->LoRaDevices[Channel].ErrorCoding = ERROR_CODING_4_5;
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_250K;
				Config->LoRaDevices[Channel].SpreadingFactor = SPREADING_6;
				Config->LoRaDevices[Channel].LowDataRateOptimize = 0;		
			}
			else if (Config->LoRaDevices[Channel].SpeedMode == 3)
			{
				// Normal mode for high speed images in 868MHz band
				Config->LoRaDevices[Channel].ImplicitOrExplicit = EXPLICIT_MODE;
				Config->LoRaDevices[Channel].ErrorCoding = ERROR_CODING_4_6;
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_250K;
				Config->LoRaDevices[Channel].SpreadingFactor = SPREADING_7;
				Config->LoRaDevices[Channel].LowDataRateOptimize = 0;		
			}
			else if (Config->LoRaDevices[Channel].SpeedMode == 2)
			{
				// Normal mode for repeater network
				// 72 byte packet is approx 1.5 seconds so needs at least 30 seconds cycle time if repeating one balloon
				Config->LoRaDevices[Channel].ImplicitOrExplicit = EXPLICIT_MODE;
				Config->LoRaDevices[Channel].ErrorCoding = ERROR_CODING_4_8;
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_62K5;
				Config->LoRaDevices[Channel].SpreadingFactor = SPREADING_8;
				Config->LoRaDevices[Channel].LowDataRateOptimize = 0;		
			}
			else if (Config->LoRaDevices[Channel].SpeedMode == 1)
			{
				// Normal mode for SSDV
				Config->LoRaDevices[Channel].ImplicitOrExplicit = IMPLICIT_MODE;
				Config->LoRaDevices[Channel].ErrorCoding = ERROR_CODING_4_5;
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_20K8;
				Config->LoRaDevices[Channel].SpreadingFactor = SPREADING_6;
				Config->LoRaDevices[Channel].LowDataRateOptimize = 0;		
			}
			else
			{
				// Normal mode for telemetry
				Config->LoRaDevices[Channel].ImplicitOrExplicit = EXPLICIT_MODE;
				Config->LoRaDevices[Channel].ErrorCoding = ERROR_CODING_4_8;
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_20K8;
				Config->LoRaDevices[Channel].SpreadingFactor = SPREADING_11;
				Config->LoRaDevices[Channel].LowDataRateOptimize = 0x08;		
			}
			
			sprintf(Keyword, "LORA_SF_%d", Channel);
			Temp = ReadInteger(fp, Keyword, 0, 0);
			if ((Temp >= 6) && (Temp <= 12))
			{
				Config->LoRaDevices[Channel].SpreadingFactor = Temp << 4;
				printf("LoRa Setting SF=%d\n", Temp);
			}

			sprintf(Keyword, "LORA_Bandwidth_%d", Channel);
			ReadString(fp, Keyword, TempString, sizeof(TempString), 0);
			if (*TempString)
			{
				printf("LoRa Setting BW=%s\n", TempString);
			}
			if (strcmp(TempString, "7K8") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_7K8;
			}
			if (strcmp(TempString, "10K4") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_10K4;
			}
			if (strcmp(TempString, "15K6") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_15K6;
			}
			if (strcmp(TempString, "20K8") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_20K8;
			}
			if (strcmp(TempString, "31K25") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_31K25;
			}
			if (strcmp(TempString, "41K7") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_41K7;
			}
			if (strcmp(TempString, "62K5") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_62K5;
			}
			if (strcmp(TempString, "125K") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_125K;
			}
			if (strcmp(TempString, "250K") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_250K;
			}
			if (strcmp(TempString, "500K") == 0)
			{
				Config->LoRaDevices[Channel].Bandwidth = BANDWIDTH_500K;
			}
			
			sprintf(Keyword, "LORA_Implicit_%d", Channel);
			if (ReadBoolean(fp, Keyword, 0, &Temp))
			{
				if (Temp)
				{
					Config->LoRaDevices[Channel].ImplicitOrExplicit = IMPLICIT_MODE;
				}
			}
			
			sprintf(Keyword, "LORA_Coding_%d", Channel);
			Temp = ReadInteger(fp, Keyword, 0, 0);
			if ((Temp >= 5) && (Temp <= 8))
			{
				Config->LoRaDevices[Channel].ErrorCoding = (Temp-4) << 1;
				printf("LoRa Setting Error Coding=%d\n", Temp);
			}

			sprintf(Keyword, "LORA_LowOpt_%d", Channel);
			if (ReadBoolean(fp, Keyword, 0, &Temp))
			{
				if (Temp)
				{
					Config->LoRaDevices[Channel].LowDataRateOptimize = 0x08;
				}
			}

			sprintf(Keyword, "LORA_Power_%d", Channel);
			Config->LoRaDevices[Channel].Power = ReadInteger(fp, Keyword, 0, PA_MAX_UK);
			printf("LoRa Channel %d power set to %02Xh\n", Channel, Config->LoRaDevices[Channel].Power);

			Config->LoRaDevices[Channel].PayloadLength = (Config->LoRaDevices[Channel].SpeedMode == 0) ? 80 : 255;
		}
		else
		{
			Config->LoRaDevices[Channel].InUse = 0;
		}
	}
	
	
	fclose(fp);
}

void SetMTX2Frequency(char *FrequencyString)
{
	float _mtx2comp;
	int _mtx2int;
	long _mtx2fractional;
	char _mtx2command[17];
	int fd;
	double Frequency;

	pinMode (NTX2B_ENABLE, OUTPUT);
	digitalWrite (NTX2B_ENABLE, 0);
	delayMilliseconds (100);
	
	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		tcgetattr(fd, &options);

		options.c_cflag &= ~CSTOPB;
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		options.c_cflag |= CS8;
		options.c_oflag &= ~ONLCR;
		options.c_oflag &= ~OPOST;

		tcsetattr(fd, TCSANOW, &options);

		delayMilliseconds (100);
		pinMode (NTX2B_ENABLE, INPUT);
		pullUpDnControl(NTX2B_ENABLE, PUD_OFF);
		delayMilliseconds (100);
		
		if (strlen(FrequencyString) < 3)
		{
			// Convert channel number to frequency
			Frequency = strtol(FrequencyString, NULL, 16) * 0.003125 + 434.05;
		}
		else
		{
			Frequency = atof(FrequencyString);
		}
		
		printf("RTTY Frequency set to %8.4fMHz\n", Frequency);
		
		_mtx2comp=(Frequency+0.0015)/6.5;
		_mtx2int=_mtx2comp;
		_mtx2fractional = ((_mtx2comp-_mtx2int)+1) * 524288;
		snprintf(_mtx2command,17,"@PRG_%02X%06lX\r",_mtx2int-1, _mtx2fractional);
		write(fd, _mtx2command, strlen(_mtx2command)); 

		delayMilliseconds (100);
		printf("MTX2 command is %s\n", _mtx2command);

		close(fd);
	}
}


void SetNTX2BFrequency(char *FrequencyString)
{
	int fd, Frequency;
	char Command[16];

	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		tcgetattr(fd, &options);

		options.c_cflag &= ~CSTOPB;
		cfsetispeed(&options, B4800);
		cfsetospeed(&options, B4800);
		options.c_cflag |= CS8;
		options.c_oflag &= ~ONLCR;
		options.c_oflag &= ~OPOST;

		tcsetattr(fd, TCSANOW, &options);

		pinMode (NTX2B_ENABLE, INPUT);
		pullUpDnControl(NTX2B_ENABLE, PUD_OFF);

		if (strlen(FrequencyString) < 3)
		{
			// Already a channel number
			Frequency = strtol(FrequencyString, NULL, 16);
		}
		else
		{
			// Convert from MHz to channel number
			Frequency = (int)((atof(FrequencyString) - 434.05) / 0.003124);
		}
		
		sprintf(Command, "%cch%02X\r", 0x80, Frequency);
		write(fd, Command, strlen(Command)); 

		printf("NTX2B-FA transmitter now set to channel %02Xh which is %8.4lfMHz\n", Frequency, (double)(Frequency) * 0.003125 + 434.05);

		close(fd);
	}
}

void SetFrequency(char *Frequency)
{
	if (NewBoard())
	{
		SetMTX2Frequency(Frequency);
	}
	else
	{
		SetNTX2BFrequency(Frequency);
	}
}

int OpenSerialPort(void)
{
	int fd;

	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		/* get the current options */
		tcgetattr(fd, &options);

		/* set raw input */
		options.c_lflag &= ~ECHO;
		options.c_cc[VMIN]  = 0;
		options.c_cc[VTIME] = 10;

		options.c_cflag |= CSTOPB;
		cfsetispeed(&options, Config.TxSpeed);
		cfsetospeed(&options, Config.TxSpeed);
		options.c_cflag |= CS8;
		options.c_oflag &= ~ONLCR;
		options.c_oflag &= ~OPOST;
	
		tcsetattr(fd, TCSANOW, &options);
	}

	return fd;
}

void SendSentence(char *TxLine)
{
	int fd;

	
	if ((fd = OpenSerialPort()) >= 0)
	{
		write(fd, TxLine, strlen(TxLine));
		
		if (close(fd) < 0)
		{
			printf("NOT Sent - error %d\n", errno);
		}
		
		if (Config.EnableTelemetryLogging)
		{
			WriteLog("telemetry.txt", TxLine);
		}
	}
	else
	{
		printf("Failed to open serial port\n");
	}
	
}

int SendImage()
{
    unsigned char Buffer[256];
    size_t Count;
    int SentSomething = 0;
	int fd;

    if (ImageFP == NULL)
    {
		if (FindAndConvertImage(RTTY_CHANNEL, SSDVFolder))
		{
			ImageFP = fopen("/home/pi/pits/tracker/snap.bin", "r");
		}
        Records = 0;
    }

    if (ImageFP != NULL)
    {
        Count = fread(Buffer, 1, 256, ImageFP);
        if (Count > 0)
        {
            printf("Record %d, %d bytes\r\n", ++Records, Count);

			if ((fd = OpenSerialPort()) >= 0)
			{
				write(fd, Buffer, Count);
				close(fd);
			}

            SentSomething = 1;
        }
        else
        {
            fclose(ImageFP);
            ImageFP = NULL;
        }
    }

    return SentSomething;
}


int main(void)
{
	int fd, ReturnCode, i;
	unsigned long Sentence_Counter = 0;
	char Sentence[100], Command[100];
	struct stat st = {0};
	struct TGPS GPS;
	pthread_t LoRaThread, APRSThread, GPSThread, DS18B20Thread, ADCThread, CameraThread, BMP085Thread, LEDThread;

	printf("\n\nRASPBERRY PI-IN-THE-SKY FLIGHT COMPUTER\n");
	printf(    "=======================================\n\n");

	if (NewBoard())
	{
		printf("RPi Model A+ or B+\n\n");
		
		Config.LED_OK = 25;
		Config.LED_Warn = 24;
		
		Config.SDA = 2;
		Config.SCL = 3;
	}
	else
	{
		printf("RPi Model A or B\n\n");

		Config.LED_OK = 4;
		Config.LED_Warn = 11;
		
		Config.SDA = 5;
		Config.SCL = 6;
	}
	
	LoadConfigFile(&Config);

	if (Config.DisableMonitor)
	{
		system("/opt/vc/bin/tvservice -off");
	}

	GPS.Time = 0.0;
	GPS.Longitude = 0.0;
	GPS.Latitude = 0.0;
	GPS.Altitude = 0;
	GPS.Satellites = 0;
	GPS.Speed = 0.0;
	GPS.Direction = 0.0;
	GPS.InternalTemperature = 0.0;
	GPS.BatteryVoltage = 0.0;
	GPS.ExternalTemperature = 0.0;
	GPS.Pressure = 0.0;
	GPS.BoardCurrent = 0.0;
	
	// Set up I/O
	if (wiringPiSetup() == -1)
	{
		exit (1);
	}

	if (*Config.Frequency)
	{
		SetFrequency(Config.Frequency);
	}

	// Switch on the radio
	pinMode (NTX2B_ENABLE, OUTPUT);
	digitalWrite (NTX2B_ENABLE, 1);
	
	// Switch on the GPS
	if (!NewBoard())
	{
		pinMode (UBLOX_ENABLE, OUTPUT);
		digitalWrite (UBLOX_ENABLE, 0);
	}
	
	if ((fd = OpenSerialPort()) < 0)
	{
		printf("Cannot open serial port - check documentation!\n");
		exit(1);
	}
	close(fd);

	// Set up DS18B20
	system("sudo modprobe w1-gpio");
	system("sudo modprobe w1-therm");
	
	// SPI for ADC
	system("gpio load spi");

	if (stat(SSDVFolder, &st) == -1)
	{
		mkdir(SSDVFolder, 0700);
	}	

	if (pthread_create(&GPSThread, NULL, GPSLoop, &GPS))
	{
		fprintf(stderr, "Error creating GPS thread\n");
		return 1;
	}

	if (*(Config.APRS_Callsign) && Config.APRS_ID && Config.APRS_Period)
	{
		if (pthread_create(&APRSThread, NULL, APRSLoop, &GPS))
		{
			fprintf(stderr, "Error creating APRS thread\n");
			return 1;
		}
	}

	if (Config.LoRaDevices[0].InUse || Config.LoRaDevices[1].InUse)
	{
		if (pthread_create(&LoRaThread, NULL, LoRaLoop, &GPS))
		{
			fprintf(stderr, "Error creating LoRa thread\n");
			return 1;
		}
	}
	
	if (pthread_create(&DS18B20Thread, NULL, DS18B20Loop, &GPS))
	{
		fprintf(stderr, "Error creating DS18B20s thread\n");
		return 1;
	}

	if (pthread_create(&ADCThread, NULL, ADCLoop, &GPS))
	{
		fprintf(stderr, "Error creating ADC thread\n");
		return 1;
	}

	if (Config.Camera)
	{
		if (pthread_create(&CameraThread, NULL, CameraLoop, &GPS))
		{
			fprintf(stderr, "Error creating camera thread\n");
			return 1;
		}
	}

	if (pthread_create(&LEDThread, NULL, LEDLoop, &GPS))
	{
		fprintf(stderr, "Error creating LED thread\n");
		return 1;
	}
	
	if (Config.EnableBMP085)
	{
		if (pthread_create(&BMP085Thread, NULL, BMP085Loop, &GPS))
		{
			fprintf(stderr, "Error creating BMP085 thread\n");
			return 1;
		}
	}
		
	while (1)
	{	
		BuildSentence(Sentence, ++Sentence_Counter, &GPS);
		
		SendSentence(Sentence);
		
		for (i=0; i< ((GPS.Altitude > Config.high) ? Config.image_packets : 1); i++)
		{
			SendImage();
		}
	}
}
