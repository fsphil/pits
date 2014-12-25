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
#include "lora.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>

// RFM98
uint8_t currentMode = 0x81;

#pragma pack(1)

struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	Seconds;
	float		Latitude;
	float		Longitude;
	uint16_t	Altitude;
};


FILE *ImageFP;
int Records, FileNumber;
struct termios options;
char *LoRaSSDVFolder="/home/pi/pits/tracker/download";

void writeRegister(int Channel, uint8_t reg, uint8_t val)
{
	unsigned char data[2];
	
	data[0] = reg | 0x80;
	data[1] = val;
	wiringPiSPIDataRW(Channel, data, 2);
}

uint8_t readRegister(int Channel, uint8_t reg)
{
	unsigned char data[2];
	uint8_t val;
	
	data[0] = reg & 0x7F;
	data[1] = 0;
	wiringPiSPIDataRW(Channel, data, 2);
	val = data[1];

    return val;
}

void setMode(int Channel, uint8_t newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF98_MODE_TX:
      // printf("Changing to Transmit Mode\n");
      writeRegister(Channel, REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
      writeRegister(Channel, REG_PA_CONFIG, Config.LoRaDevices[Channel].Power);
      writeRegister(Channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister(Channel, REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(Channel, REG_LNA, LNA_MAX_GAIN);  // LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(Channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      // printf("Changing to Receive Continuous Mode\n");
      break;
    case RF98_MODE_SLEEP:
      writeRegister(Channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      // printf("Changing to Sleep Mode\n"); 
      break;
    case RF98_MODE_STANDBY:
      writeRegister(Channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      // printf("Changing to Standby Mode\n");
      break;
    default: return;
  } 
  
	if(newMode != RF98_MODE_SLEEP)
	{
	// Wait for mode change
		while(digitalRead(5) == 0)
		{
		} 
	}
	
	// printf("Mode Change Done\n");
  
	return;
}
 
void setLoRaMode(int Channel)
{
	double Frequency;
	unsigned long FrequencyValue;

	printf("Setting LoRa Mode\n");
	setMode(Channel, RF98_MODE_SLEEP);
	writeRegister(Channel, REG_OPMODE, 0x80);

	setMode(Channel, RF98_MODE_SLEEP);
  
	if (sscanf(Config.LoRaDevices[Channel].Frequency, "%lf", &Frequency))
	{
		FrequencyValue = (unsigned long)(Frequency * 7110656 / 434);
		printf("FrequencyValue = %06Xh\n", FrequencyValue);
		writeRegister(Channel, 0x06, (FrequencyValue >> 16) & 0xFF);		// Set frequency
		writeRegister(Channel, 0x07, (FrequencyValue >> 8) & 0xFF);
		writeRegister(Channel, 0x08, FrequencyValue & 0xFF);
	}

	printf("Mode = %d\n", readRegister(Channel, REG_OPMODE));
}

void setupRFM98(int Channel)
{
	if (Config.LoRaDevices[Channel].InUse)
	{
		// initialize the pins
		pinMode(Config.LoRaDevices[Channel].DIO0, INPUT);
		pinMode(Config.LoRaDevices[Channel].DIO5, INPUT);

		if (wiringPiSPISetup(Channel, 500000) < 0)
		{
			fprintf(stderr, "Failed to open SPI port.  Try loading spi library with 'gpio load spi'");
			exit(1);
		}

		// LoRa mode 
		setLoRaMode(Channel);

		writeRegister(Channel, REG_MODEM_CONFIG, Config.LoRaDevices[Channel].ImplicitOrExplicit | Config.LoRaDevices[Channel].ErrorCoding | Config.LoRaDevices[Channel].Bandwidth);
		writeRegister(Channel, REG_MODEM_CONFIG2, Config.LoRaDevices[Channel].SpreadingFactor | CRC_ON);
		writeRegister(Channel, REG_MODEM_CONFIG3, 0x04 | Config.LoRaDevices[Channel].LowDataRateOptimize);									// 0x04: AGC sets LNA gain
		writeRegister(Channel, REG_DETECT_OPT, (Config.LoRaDevices[Channel].SpreadingFactor == SPREADING_6) ? 0x05 : 0x03);					// 0x05 For SF6; 0x03 otherwise
		writeRegister(Channel, REG_DETECTION_THRESHOLD, (Config.LoRaDevices[Channel].SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);		// 0x0C for SF6, 0x0A otherwise
		
		writeRegister(Channel, REG_PAYLOAD_LENGTH, Config.LoRaDevices[Channel].PayloadLength);
		writeRegister(Channel, REG_RX_NB_BYTES, Config.LoRaDevices[Channel].PayloadLength);

		writeRegister(Channel, REG_FIFO_ADDR_PTR, 0);		// woz readRegister(Channel, REG_FIFO_RX_BASE_AD));   

		// writeRegister(Channel, REG_DIO_MAPPING_1,0x40);
		writeRegister(Channel, REG_DIO_MAPPING_2,0x00);
	}
}

void sendData(int Channel, unsigned char *buffer, int Length)
{
	unsigned char data[257];
	int i;
	
	// printf("Sending %d bytes\n", Length);
  
	setMode(Channel, RF98_MODE_STANDBY);
	
	writeRegister(Channel, REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 maps DIO0 to TxDone

	writeRegister(Channel, REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
	writeRegister(Channel, REG_FIFO_ADDR_PTR, 0x00); 
  
	data[0] = REG_FIFO | 0x80;
	for (i=0; i<Length; i++)
	{
		data[i+1] = buffer[i];
	}
	wiringPiSPIDataRW(Channel, data, Length+1);

	// printf("Set Tx Mode\n");
  
	writeRegister(Channel, REG_PAYLOAD_LENGTH, Length);

	// go into transmit mode
	setMode(Channel, RF98_MODE_TX);
	
	Config.LoRaDevices[Channel].LoRaMode = lmSending;
}


int BuildLoRaSentence(char *TxLine, int Channel, struct TGPS *GPS)
{
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	char TimeBuffer1[12], TimeBuffer2[10], ExtraFields[80];
	
	Config.Channels[LORA_CHANNEL+Channel].SentenceCounter++;
	
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
	
	ExtraFields[0] = '\0';
	
	if (Config.EnableBMP085)
	{
		sprintf(ExtraFields, ",%.1f,%.0f", GPS->ExternalTemperature, GPS->Pressure);
	}
	
    sprintf(TxLine, "$$%s,%d,%s,%7.5lf,%7.5lf,%05.5u,%d,%d,%d,%d,%d,%s",
            Config.Channels[LORA_CHANNEL+Channel].PayloadID,
            Config.Channels[LORA_CHANNEL+Channel].SentenceCounter,
			TimeBuffer2,
            GPS->Latitude,
            GPS->Longitude,
            GPS->Altitude,
			(GPS->Speed * 13) / 7,
			GPS->Direction,
			GPS->Satellites,
			Config.LoRaDevices[Channel].GroundCount,
			Config.LoRaDevices[Channel].AirCount,
			Config.LoRaDevices[Channel].LastCommand);

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
	
	return strlen(TxLine) + 1;
}

int BuildLoRaPositionPacket(char *TxLine, int Channel, struct TGPS *GPS)
{
	int OurID;
	struct TBinaryPacket BinaryPacket;
	
	OurID = Config.LoRaDevices[Channel].Slot;
	
	Config.Channels[LORA_CHANNEL+Channel].SentenceCounter++;

	BinaryPacket.PayloadIDs = 0xC0 | (OurID << 3) | OurID;
	BinaryPacket.Counter = Config.Channels[LORA_CHANNEL+Channel].SentenceCounter;
	BinaryPacket.Seconds = GPS->Seconds;
	BinaryPacket.Latitude = GPS->Latitude;
	BinaryPacket.Longitude = GPS->Longitude;
	BinaryPacket.Altitude = GPS->Altitude;

	memcpy(TxLine, &BinaryPacket, sizeof(BinaryPacket));
	
	return sizeof(struct TBinaryPacket);
}

int SendLoRaImage(int Channel)
{
    unsigned char Buffer[256];
    size_t Count;
    int SentSomething = 0;

    if (ImageFP == NULL)
    {
		if (FindAndConvertImage(LORA_CHANNEL + Channel, LoRaSSDVFolder))
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

			sendData(Channel, Buffer+1, 255);
			
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

int TimeToSendOnThisChannel(int Channel, struct TGPS *GPS)
{
	long CycleSeconds;
	
	if (Config.LoRaDevices[Channel].CycleTime == 0)
	{
		// Not using time to decide when we can send
		return 1;
	}
	
	// Can't send till we have the time!
	if (GPS->Satellites > 0)
	{
		// Can't Tx twice at the same time
		if (GPS->Seconds != Config.LoRaDevices[Channel].LastTxAt)
		{
			CycleSeconds = GPS->Seconds % Config.LoRaDevices[Channel].CycleTime;
	
			if (CycleSeconds == Config.LoRaDevices[Channel].Slot)
			{
				Config.LoRaDevices[Channel].LastTxAt = GPS->Seconds;
				Config.LoRaDevices[Channel].SendRepeatedPacket = 0;
				return 1;
			}

			if (Config.LoRaDevices[Channel].PacketRepeatLength && (CycleSeconds == Config.LoRaDevices[Channel].RepeatSlot))
			{
				Config.LoRaDevices[Channel].LastTxAt = GPS->Seconds;
				Config.LoRaDevices[Channel].SendRepeatedPacket = 1;
				return 1;
			}
			
			if (Config.LoRaDevices[Channel].UplinkRepeatLength && (CycleSeconds == Config.LoRaDevices[Channel].UplinkSlot))
			{
				Config.LoRaDevices[Channel].LastTxAt = GPS->Seconds;
				Config.LoRaDevices[Channel].SendRepeatedPacket = 2;
				return 1;
			}
			
		}
	}
	
	return 0;
}

void startReceiving(int Channel)
{
	if (Config.LoRaDevices[Channel].InUse)
	{
	
		writeRegister(Channel, REG_DIO_MAPPING_1, 0x00);		// 00 00 00 00 maps DIO0 to RxDone
	
		writeRegister(Channel, REG_FIFO_RX_BASE_AD, 0);
		writeRegister(Channel, REG_FIFO_ADDR_PTR, 0);
	  
		// Setup Receive Continuous Mode
		setMode(Channel, RF98_MODE_RX_CONTINUOUS); 
		
		Config.LoRaDevices[Channel].LoRaMode = lmListening;
	}
}

int receiveMessage(int Channel, unsigned char *message)
{
	int i, Bytes, currentAddr, x;
	unsigned char data[257];

	Bytes = 0;
	
	x = readRegister(Channel, REG_IRQ_FLAGS);
  
	// clear the rxDone flag
	writeRegister(Channel, REG_IRQ_FLAGS, 0x40); 
   
	// check for payload crc issues (0x20 is the bit we are looking for
	if((x & 0x20) == 0x20)
	{
		// CRC Error
		writeRegister(Channel, REG_IRQ_FLAGS, 0x20);		// reset the crc flags
		Config.LoRaDevices[Channel].BadCRCCount++;
	}
	else
	{
		currentAddr = readRegister(Channel, REG_FIFO_RX_CURRENT_ADDR);
		Bytes = readRegister(Channel, REG_RX_NB_BYTES);

		// ChannelPrintf(Channel,  9, 1, "Packet   SNR = %4d   ", (char)(readRegister(Channel, REG_PACKET_SNR)) / 4);
		// ChannelPrintf(Channel, 10, 1, "Packet  RSSI = %4d   ", readRegister(Channel, REG_PACKET_RSSI) - 157);
		// ChannelPrintf(Channel, 11, 1, "Freq. Error = %4.1lfkHz ", FrequencyError(Channel) / 1000);

		writeRegister(Channel, REG_FIFO_ADDR_PTR, currentAddr);   
		
		data[0] = REG_FIFO;
		wiringPiSPIDataRW(Channel, data, Bytes+1);
		for (i=0; i<=Bytes; i++)
		{
			message[i] = data[i+1];
		}
		
		message[Bytes] = '\0';
	} 

	// Clear all flags
	writeRegister(Channel, REG_IRQ_FLAGS, 0xFF); 
  
	return Bytes;
}

void CheckForPacketOnListeningChannels(void)
{
	int Channel;
	
	for (Channel=0; Channel<=1; Channel++)
	{
		if (Config.LoRaDevices[Channel].InUse)
		{
			if (Config.LoRaDevices[Channel].LoRaMode == lmListening)
			{
				if (digitalRead(Config.LoRaDevices[Channel].DIO0))
				{
					unsigned char Message[256];
					int Bytes;
					
					Bytes = receiveMessage(Channel, Message);
					printf ("Rx %d bytes\n", Bytes);
					
					if (Bytes > 0)
					{
						if (Message[0] == '$')
						{
							char Payload[32];

							printf("Balloon message\n");
							if (sscanf(Message+2, "%32[^,]", Payload) == 1)
							{
								if (strcmp(Payload, Config.Channels[LORA_CHANNEL+Channel].PayloadID) != 0)
								{
									// printf ("%s\n", Message);
							
									strcpy(Config.LoRaDevices[Channel].PacketToRepeat, Message);
									Config.LoRaDevices[Channel].PacketRepeatLength = strlen(Message);
							
									Config.LoRaDevices[Channel].AirCount++;

									Message[strlen(Message)] = '\0';
								}
							}
						}
						else if ((Message[0] & 0xC0) == 0xC0)
						{
							char Payload[32];
							int SourceID, OurID;
							
							OurID = Config.LoRaDevices[Channel].Slot;
							SourceID = Message[0] & 0x07;
							
							if (SourceID == OurID)
							{
								printf("Balloon Binary Message - ignored\n");
							}
							else
							{
								printf("Balloon Binary Message from sender %d\n", SourceID);
								
								// Replace the sender ID with ours
								Message[0] = Message[0] & 0xC7 | (OurID << 3);
								Config.LoRaDevices[Channel].PacketRepeatLength = sizeof(struct TBinaryPacket);
								memcpy(Config.LoRaDevices[Channel].PacketToRepeat, Message, Config.LoRaDevices[Channel].PacketRepeatLength);
							
								Config.LoRaDevices[Channel].AirCount++;
							}
						}
						else if ((Message[0] & 0xC0) == 0x80)
						{
							int SenderID, TargetID, OurID;
							
							TargetID = Message[0] & 0x07;
							SenderID = (Message[0] >> 3) & 0x07;
							OurID = Config.LoRaDevices[Channel].Slot;

							printf("Uplink from %d to %d Message %s\n",
									SenderID,
									TargetID,
									Message+1);
									
							if (TargetID == OurID)
							{
								printf("Message was for us!\n");
								strcpy(Config.LoRaDevices[Channel].LastCommand, Message+1);
								printf("Message is '%s'\n", Config.LoRaDevices[Channel].LastCommand);
								Config.LoRaDevices[Channel].GroundCount++;
							}
							else
							{
								printf("Message was for another balloon\n");
								Message[0] = Message[0] & 0xC7 | (OurID << 3);
								Config.LoRaDevices[Channel].UplinkRepeatLength = sizeof(struct TBinaryPacket);
								memcpy(Config.LoRaDevices[Channel].UplinkPacket, Message, Config.LoRaDevices[Channel].UplinkRepeatLength);
							}
						}
						else
						{
							printf("Unknown message %02Xh\n", Message[0]);
						}
					}
				}
			}
		}
	}
}

int CheckForFreeChannel(struct TGPS *GPS)
{
	int Channel;
	
	for (Channel=0; Channel<=1; Channel++)
	{
		if (Config.LoRaDevices[Channel].InUse)
		{
			if ((Config.LoRaDevices[Channel].LoRaMode != lmSending) || digitalRead(Config.LoRaDevices[Channel].DIO0))
			{
				// Either not sending, or was but now it's sent.  Clear the flag if we need to
				if (Config.LoRaDevices[Channel].LoRaMode == lmSending)
				{
					// Clear that IRQ flag
					writeRegister(Channel, REG_IRQ_FLAGS, 0x08); 
					Config.LoRaDevices[Channel].LoRaMode = lmIdle;
				}
				// else if ((Channel == 1) && (Config.LoRaDevices[Channel].CycleTime == 0))
				// {
					// // Get here first time that channel 1 is in use
					// // Add a short delay to put the 2 channels out of sync, to make things easier at the rx end
					// delay(2000);
				// }
				
				// Mow we test to see if we're doing TDM or not
				// For TDM, if it's not a slot that we send in, then we should be in listening mode
				// Otherwise, we just send
				
				if (TimeToSendOnThisChannel(Channel, GPS))
				{
					// Either sending continuously, or it's our slot to send in
					// printf("Channel %d is free\n", Channel);
					
					return Channel;
				}
				else if (Config.LoRaDevices[Channel].CycleTime > 0)
				{
					// TDM system and not time to send, so we can listen
					if (Config.LoRaDevices[Channel].LoRaMode == lmIdle)
					{
						startReceiving(Channel);
					}
				}
			}
		}
	}
	
	return -1;
}
	
void *LoRaLoop(void *some_void_ptr)
{
	int ReturnCode, ImagePacketCount, fd, Channel;
	unsigned long Sentence_Counter = 0;
	char Sentence[100], Command[100];
	struct stat st = {0};
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;

	if (stat(LoRaSSDVFolder, &st) == -1)
	{
		mkdir(LoRaSSDVFolder, 0700);
	}	

	for (Channel=0; Channel<2; Channel++)
	{
		setupRFM98(Channel);
		startReceiving(Channel);
	}

	ImagePacketCount = 0;
	
	while (1)
	{	
		int MaxImagePackets, Channel;
		
		MaxImagePackets = (GPS->Altitude > Config.high) ? Config.image_packets : 1;
		
		delay(5);								// To stop this loop gobbling up CPU

		CheckForPacketOnListeningChannels();
		
		Channel = CheckForFreeChannel(GPS);		// 0 or 1 if there's a free channel and we should be sending on that channel now
		
		if (Channel >= 0)
		{
			// delay(20);

			if (Config.LoRaDevices[Channel].SendRepeatedPacket == 2)
			{
				printf("Repeating uplink packet of %d bytes\n", Config.LoRaDevices[Channel].UplinkRepeatLength);
				
				sendData(Channel, Config.LoRaDevices[Channel].UplinkPacket, Config.LoRaDevices[Channel].UplinkRepeatLength);
				
				Config.LoRaDevices[Channel].UplinkRepeatLength = 0;
			}
			else if (Config.LoRaDevices[Channel].SendRepeatedPacket == 1)
			{
				printf("Repeating balloon packet of %d bytes\n", Config.LoRaDevices[Channel].PacketRepeatLength);
				
				sendData(Channel, Config.LoRaDevices[Channel].PacketToRepeat, Config.LoRaDevices[Channel].PacketRepeatLength);
				
				Config.LoRaDevices[Channel].PacketRepeatLength = 0;
			}
			else			
			// if (ImagePacketCount >= MaxImagePackets)
			{
				int PacketLength;
				
				ImagePacketCount = 0;

				if (Config.LoRaDevices[Channel].Binary)
				{
					PacketLength = BuildLoRaPositionPacket(Sentence, Channel, GPS);
					printf("LoRa: Binary packet %d bytes\n", PacketLength);
				}
				else
				{
					PacketLength = BuildLoRaSentence(Sentence, Channel, GPS);
					printf("LoRa: %s", Sentence);
				}
								
				sendData(Channel, Sentence, PacketLength);		
			}
			/*
			else
			{
				if (SendLoRaImage(Channel))
				{
				}
				ImagePacketCount++;
			}
			*/
		}
	}
}
