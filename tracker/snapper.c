#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <gertboard.h>

#include "gps.h"
#include "misc.h"

int TimeTillImageCompleted(int Channel)
{
	// First, if we aren't sending a file, then we need a new one NOW!
	if (Config.Channels[Channel].ImageFP == NULL || 
	    (Config.Channels[Channel].SSDVTotalRecords == 0))
	{
		printf ("Convert image now for channel %d!\n", Channel);
		return 0;
	}

	// If we're on the last packet, convert anyway
	if (Config.Channels[Channel].SSDVRecordNumber >= (Config.Channels[Channel].SSDVTotalRecords-1))
	{
		return 0;
	}
		
	return (Config.Channels[Channel].SSDVTotalRecords - Config.Channels[Channel].SSDVRecordNumber) * 256 * 10 / Config.Channels[Channel].BaudRate;
}

void FindAndConvertImage(int Channel)
{
	static char *SubFolder[4] = {"RTTY", "APRS", "LORA0", "LORA1"};
	size_t LargestFileSize;
	char LargestFileName[100], FileName[100], CommandLine[200];
	DIR *dp;
	struct dirent *ep;
	struct stat st;
	char *SSDVFolder;
	
	LargestFileSize = 0;
	SSDVFolder = Config.Channels[Channel].SSDVFolder;
	
	dp = opendir(SSDVFolder);
	if (dp != NULL)
	{
		while (ep = readdir (dp))
		{
			if (strstr(ep->d_name, ".jpg") != NULL)
			{
				sprintf(FileName, "%s/%s", SSDVFolder, ep->d_name);
				stat(FileName, &st);
				if (st.st_size > LargestFileSize)
				{
					LargestFileSize = st.st_size;
					strcpy(LargestFileName, FileName);
				}
			}
		}
		(void) closedir (dp);
	}

	if (LargestFileSize > 0)
	{
		char Date[20], SavedImageFolder[100], SSDVScriptName[200];
		time_t now;
		struct tm *t;
		FILE *fp;
		
		printf("Found file %s to convert\n", LargestFileName);
		
		// Now create a script to convert the file
		Config.Channels[Channel].SSDVFileNumber++;
		Config.Channels[Channel].SSDVFileNumber = Config.Channels[Channel].SSDVFileNumber & 255;
		
		// Set name of SSDV output file
		sprintf(Config.Channels[Channel].SSDVFileName, "/home/pi/pits/tracker/snap_%d_%d.bin", Channel, Config.Channels[Channel].SSDVFileNumber);
		
		// Set name of SSDV script file
		sprintf(SSDVScriptName, "/home/pi/pits/tracker/convert_%d", Channel);
				
		// Create SSDV script file
		if ((fp = fopen(SSDVScriptName, "wt")) != NULL)
		{
			fprintf(fp, "ssdv -e -c %s -i %d %s %s\n", Config.Channels[Channel].PayloadID, Config.Channels[Channel].SSDVFileNumber, LargestFileName, Config.Channels[Channel].SSDVFileName);
			fprintf(fp, "mkdir -p %s/$1\n", SSDVFolder);
			fprintf(fp, "mv %s/*.jpg %s/$1\n", SSDVFolder, SSDVFolder);
			fclose(fp);
			chmod(SSDVScriptName, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH); 
		}
		Config.Channels[Channel].NextSSDVFileReady = 1;
	}
}

void *CameraLoop(void *some_void_ptr)
{
	int width, height;
	struct TGPS *GPS;
	char filename[100];
	int Channel;
	FILE *fp;

	GPS = (struct TGPS *)some_void_ptr;
	
	for (Channel=0; Channel<5; Channel++)
	{
		Config.Channels[Channel].TimeSinceLastImage = Config.Channels[Channel].ImagePeriod;
	}

	while (1)
	{
		for (Channel=0; Channel<5; Channel++)
		{
			if (Config.Channels[Channel].Enabled && (Config.Channels[Channel].ImagePackets > 0))
			{
				// Channel using SSDV
				
				if (++Config.Channels[Channel].TimeSinceLastImage >= Config.Channels[Channel].ImagePeriod)
				{
					// Time to take a photo on this channel

					Config.Channels[Channel].TimeSinceLastImage = 0;
					
					if (GPS->Altitude >= Config.SSDVHigh)
					{
						width = Config.Channels[Channel].ImageWidthWhenHigh;
						height = Config.Channels[Channel].ImageHeightWhenHigh;
					}
					else
					{
						width = Config.Channels[Channel].ImageWidthWhenLow;
						height = Config.Channels[Channel].ImageHeightWhenLow;
					}

					// Create name of file
					sprintf(filename, "/home/pi/pits/tracker/take_pic_%d", Channel);
					
					// Leave it alone if it exists (this means that the photo has not been taken yet)
					if (access(filename, F_OK ) == -1)
					{				
						// Doesn't exist, so create it.  Script will run it next time it checks
						if ((fp = fopen(filename, "wt")) != NULL)
						{
							if (Channel == 4)
							{
								// Full size images are saved in dated folder names
								fprintf(fp, "mkdir -p %s/$2\n", Config.Channels[Channel].SSDVFolder);
								fprintf(fp, "raspistill -w %d -h %d -t 3000 -ex auto -mm matrix -o %s/$2/$1.jpg\n", width, height, Config.Channels[Channel].SSDVFolder);
							}
							else
							{
								fprintf(fp, "raspistill -w %d -h %d -t 3000 -ex auto -mm matrix -o %s/$1.jpg\n", width, height, Config.Channels[Channel].SSDVFolder);
							}
							
							fclose(fp);
							chmod(filename, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH); 
						}
					}
					Config.Channels[Channel].ImagesRequested++;
				}
				
				// Now check if we need to convert the "best" image before the current SSDV file is fully sent
				
				if (Config.Channels[Channel].ImagesRequested && !Config.Channels[Channel].NextSSDVFileReady)
				{
					if (TimeTillImageCompleted(Channel) < 5)
					{
						FindAndConvertImage(Channel);
					}
				}
			}
		}
		
		sleep(1);
	}
}


