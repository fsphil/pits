// Types

struct TGPS
{
	long Time;						// Time as read from GPS, as an integer but 12:13:14 is 121314
	long Seconds;					// Time in seconds since midnight
	float Longitude, Latitude;
	unsigned int Altitude;
	unsigned int Satellites;
	int Speed;
	int Direction;
	float InternalTemperature;
	float BatteryVoltage;
	float ExternalTemperature;
	float Pressure;
	unsigned int BoardCurrent;
} GPS;


// functions

void *GPSLoop(void *some_void_ptr);


