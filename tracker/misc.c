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
#include "misc.h"

int FileNumber;

char Hex(char Character)
{
	char HexTable[] = "0123456789ABCDEF";
	
	return HexTable[Character];
}

void WriteLog(char *FileName, char *Buffer)
{
	FILE *fp;
	
	if ((fp = fopen(FileName, "at")) != NULL)
	{
		fputs(Buffer, fp);
		fclose(fp);
	}
}

int NewBoard(void)
{
	FILE *cpuFd ;
	char line [120] ;
	char *c ;
	static int  boardRev = -1 ;

	if (boardRev < 0)
	{
		if ((cpuFd = fopen ("/proc/cpuinfo", "r")) != NULL)
		{
			while (fgets (line, 120, cpuFd) != NULL)
				if (strncmp (line, "Revision", 8) == 0)
					break ;

			fclose (cpuFd) ;

			if (strncmp (line, "Revision", 8) == 0)
			{
				printf ("RPi %s", line);
				boardRev = ((strstr(line, "0010") != NULL) || (strstr(line, "0012") != NULL));	// B+ or A+
			}
		}
	}
	
	return boardRev;
}

void StartNewFileIfNeeded(int Channel)
{
    if (Config.Channels[Channel].ImageFP == NULL)
    {
		// Not currently sending a file
		if (Config.Channels[Channel].NextSSDVFileReady)
		{
			// Script has been created, but possibly not run yet
			// So just try to open the file
			
			if ((Config.Channels[Channel].ImageFP = fopen(Config.Channels[Channel].SSDVFileName, "r")) != NULL)
			{
				// That workd so let's get the file size so we can monitor progress
				fseek(Config.Channels[Channel].ImageFP, 0L, SEEK_END);
				Config.Channels[Channel].SSDVTotalRecords = ftell(Config.Channels[Channel].ImageFP) / 256;		// SSDV records are 256 bytes
				fseek(Config.Channels[Channel].ImageFP, 0L, SEEK_SET);				
				
				// Set record counter back to zero
				Config.Channels[Channel].SSDVRecordNumber = 0;
				
				// And clear the flag so that the script can be recreated later
				Config.Channels[Channel].NextSSDVFileReady = 0;
			}
		}
	}
}
