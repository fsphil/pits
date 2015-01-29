#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <gertboard.h>

#include "gps.h"
#include "misc.h"

void *CameraLoop(void *some_void_ptr)
{
	int width, height;
	struct TGPS *GPS;
	char filename[100];
	int Channel;
	FILE *fp;

	GPS = (struct TGPS *)some_void_ptr;
	
	while (1)
	{
		for (Channel=0; Channel<5; Channel++)
		{
			if (Config.Channels[Channel].ImagePackets > 0)
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
				}
			}
		}
		
		sleep(1);
	}
}


