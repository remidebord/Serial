#include <stdio.h>
#include "../inc/serial.h"

// Create serial port
serial serial;

int main(int argc, char** argv)
{
	char buffer[256] = {0};
	int i, length = 0;
	
	// Open serial port ("COM3", "/dev/ttyUSB0")
	serial.Open("COM3", 9600, 8, NO, 1);
	
	while(1)
	{
		// Wait character
		length = serial.Read(buffer);
		
		if(length)
		{
			for(i = 0; i < length; i++)
			{
				printf("%.2X ", buffer[i]);
				//printf("%c", buffer[i]);
			}
			
			printf("\n");
			
			// Send data
			serial.Write(buffer, length);
		}
	}
	
	// Close serial port
	serial.Close();
	
	return 0;
}
