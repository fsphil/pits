// Globals

#include <termios.h>

// Structure for the LoRa devices.  The LoRa board has 1 or 2 LoRa modems.
// Current labellibng is "1" for SPI channel 0 and "2" for SPI channel 1
// If the board is used with PITS then only "2" can be used (though both can be populated)

typedef enum {lmIdle, lmListening, lmSending} tLoRaMode;


struct TLoRaDevice
{
	int InUse;
	int DIO0;
	int DIO5;
	char Frequency[8];
	int SpeedMode;
	int Power;
	int PayloadLength;
	int ImplicitOrExplicit;
	int ErrorCoding;
	int Bandwidth;
	int SpreadingFactor;
	int LowDataRateOptimize;
	int CycleTime;
	int Slot;
	int RepeatSlot;
	int UplinkSlot;
	int Binary;
	int LastTxAt;
	int LastRxAt;
	int AirCount;
	int GroundCount;
	int BadCRCCount;
	char LastCommand[128];
	unsigned char PacketToRepeat[256];
	unsigned char UplinkPacket[256];
	int PacketRepeatLength;
	int UplinkRepeatLength;
	int SendRepeatedPacket;
	tLoRaMode LoRaMode;
};

// Structure for all possible radio devices
// 0 is RTTY
// 1 is APRS
// 2/3 are for LoRa
struct TChannel
{
	unsigned int SentenceCounter;
	char PayloadID[16];
	int SendTelemetry;
	int SendImages;
	char SSDVFolder[100];
};

#define RTTY_CHANNEL 0
#define APRS_CHANNEL 1
#define LORA_CHANNEL 2

struct TConfig
{
	int DisableMonitor;
	int Camera;
	int low_width;
	int low_height;
	int high;
	int high_width;
	int high_height;
	int image_packets;
	int EnableBMP085;
	int EnableGPSLogging;
	int EnableTelemetryLogging;
	int LED_OK;
	int LED_Warn;
	
	// GPS Settings
	int SDA;
	int SCL;
	char GPSDevice[32];
	
	// RTTY Settings
	int DisableRTTY;
	char Frequency[8];
	speed_t TxSpeed;
	
	char APRS_Callsign[16];
	int APRS_ID;
	int APRS_Period;
		
	struct TLoRaDevice LoRaDevices[2];
	
	struct TChannel Channels[4];
};

extern struct TConfig Config;

char Hex(char Character);
void WriteLog(char *FileName, char *Buffer);
int FindAndConvertImage(int RadioChannel, char *SSDVFolder);
