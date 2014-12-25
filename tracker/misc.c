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

int FindAndConvertImage(int RadioChannel, char *SSDVFolder)
{
	size_t LargestFileSize;
	char LargestFileName[100], FileName[100], CommandLine[200];
	DIR *dp;
	struct dirent *ep;
	struct stat st;
	int Done;
	
	LargestFileSize = 0;
	Done = 0;
	
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
		char Date[20], SavedImageFolder[100];
		time_t now;
		struct tm *t;
		
		printf("Found file %s to convert\n", LargestFileName);
		
		// Now convert the file
		FileNumber++;
		FileNumber = FileNumber & 255;
		sprintf(CommandLine, "ssdv -e -c %s -i %d %s /home/pi/pits/tracker/snap.bin", Config.Channels[RadioChannel].PayloadID, FileNumber, LargestFileName);
		system(CommandLine);
		
		// And move those pesky image files
		now = time(NULL);
		t = localtime(&now);
		strftime(Date, sizeof(Date)-1, "%d_%m_%Y", t);		

		sprintf(SavedImageFolder, "%s/%s", SSDVFolder, Date);
		if (stat(SavedImageFolder, &st) == -1)
		{
			mkdir(SavedImageFolder, 0777);
		}
		system(CommandLine);
		sprintf(CommandLine, "mv %s/*.jpg %s", SSDVFolder, SavedImageFolder);
		system(CommandLine);

		Done = 1;
	}
	
	return (LargestFileSize > 0);
}
