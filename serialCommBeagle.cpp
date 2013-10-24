/*
 * File: serialCommBeagle.cpp
 * 
 * Author: Mike Mozingo
 * 
 * Desctiption: Demonstrates functinoality required to read and write to a serial port
 * on the beagle bone. In linux, serial ports can be accessed through files, in this
 * case we are accessing UART1 through ttyO1. (Oh-one, not zero-one)
 * 
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>


int main()
{
	int fd; // file handle

	char buff[20]; // read buffer

	volatile int rd,wr; // read character count and write character count

	fd=open("/dev/ttyO1",(O_RDWR | O_NOCTTY | O_NDELAY)); // open the serial port file
							      // O_RDWR : set to read and write
							      // O_NOCITY : specify that this is not the 'controlling terminal'
							      // O_NODELAY : we don't care about DCD signals

	if (fd == -1)
	{
		perror("open_port: Unable to open /dev/ttyO1 - ");
	}
	
	else
	{
		fcntl(fd, F_SETFL,0); // Used with O_NODELAY, supposed to set read to blocking, doesn't
		printf("Port 1 has been successfully opened and %d is the file description\n", fd);
	}

	printf("Printing ATZ to the serial port\n");
	wr = write(fd,"ATZ",3); // write(filehandle, character array, bytecount)
	printf("Trying to read...\n");
	usleep(10); // microsecond sleep

	while(1)
	{
		rd = read(fd,buff,1); // read(filehandle, read buffer, read-bytecount)
		if (rd > 0) // check if bytes read
		{
			printf("Read %d bytes\n",rd); // yay!
//			return 0;
		}
		usleep(10); // microsecond sleep
	}
	
	close(fd);

	return 0;
}
