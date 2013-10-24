/*
 * BBBSerial.cpp
 *
 *  Created on: Oct 7, 2013
 *      Author: michaeldarling
 */

#include "BBBSerial.h"
#include <iostream>

BBBSerial::BBBSerial()
{
	// To enable UART1 on BBB:
	// # These instructions describe how to enable UART 1 on startup
	// # Reference:
	// # http://learn.adafruit.com/introduction-to-the-beaglebone-black-device-tree/exporting-and-unexporting-an-overlay
	//
	// mkdir /mnt/boot
	// mount /dev/mmcblk0p1 /mnt/boot
	// nano /mnt/boot/uEnv.txt
	// #add this to the end of the single line of uEnv.txt:
	// capemgr.enable_partno=BB-UART1

	// Initialize header message, checksum, and payload length members
	hdr[0] = 'D';
	hdr[1] = 'A';
	hdr[2] = 'T';
	hdr[3] = 'A';
	chk_hdr = 'D' ^ 'A' ^ 'T' ^ 'A';
	payload_len = 6;

	// initialize read count
	rd=0;

	read_timeout = 0; //milliseconds



	// Open the serial port
	fd = open("/dev/ttyO1",(O_RDWR | O_NOCTTY | O_NDELAY));

	struct termios settings;
	tcgetattr(fd, &settings);
	cfmakeraw(&settings);
	cfsetispeed(&settings,BBBSERIAL_BAUD);
	cfsetospeed(&settings,BBBSERIAL_BAUD);

	if (tcsetattr(fd, TCSANOW, &settings) != 0)
		std::cerr << "failed to set serial port settings" << std::endl;


	if (fd == -1)
		perror("open_port: Unable to open /dev/ttyO1 = ");

}

BBBSerial::~BBBSerial()
{
	// Close the file descriptor
	close(fd);
}


bool BBBSerial::readBytes(uint8_t &msg)
{
	// Try and read until timeout is reached

	clock_t t;
	unsigned int telapsed;
	t = clock();

	do
	{
		rd = read(fd,&msg,sizeof(msg));

		if (rd > 0)
			return true;

		telapsed = ((clock() - t) / CLOCKS_PER_SEC * 1000); // milliseconds
	}
	while (telapsed < read_timeout);

	return false;
}

void BBBSerial::writeBytes(const uint8_t &wt)
{
	write(fd,&wt,sizeof(wt));
}

int BBBSerial::writeData(const std::vector<double> &state)
{
	// Send a message containing 6-DOF state info using message protocol expected by APM

	// Check for a data request
	int rchk = BBBSerial::checkRequest();
	if (rchk < 1)
		return rchk;

	union {				// message payload element as float and bytes
		uint8_t b[4];
		float f;
	} pld;


	// make sure a float is 4 bytes on the target machine
	assert(sizeof(float) == 4);


	// send header message and initialize checksum
	for (int i=0; i<sizeof(hdr); i++)
	{
		BBBSerial::writeBytes(hdr[i]);
	}
	uint8_t chk = chk_hdr;

	// send the payload length = 6
	// BBBSerial::writeBytes(payload_len);
	// chk ^= payload_len;

	//send the payload
	for (int i=0; i<payload_len; i++)
	{
		pld.f = (float) state[i];  			// Typecast state to a 4-byte float
		BBBSerial::writeBytes(pld.b[0]);	// Write each of the four bytes representing the float
		BBBSerial::writeBytes(pld.b[1]);
		BBBSerial::writeBytes(pld.b[2]);
		BBBSerial::writeBytes(pld.b[3]);

		// update checksum
		chk ^= pld.b[0] ^ pld.b[1] ^ pld.b[2] ^ pld.b[3];
	}

	// Send a fake LED bitmask to preserve message format
	BBBSerial::writeBytes(0xFF);
	chk ^= 0xFF;

	// send the checksum
	BBBSerial::writeBytes(chk);

	return true;
}

int BBBSerial::checkRequest()
{
	// Return vals:
	//  -1 --  no data in buffer
	//   0 --  no request
	//   1 --  data requested

	uint8_t msg;

	if (BBBSerial::readBytes(msg)) {
		if (msg == 'H')
			return 1;
		else
			return 0;
	} else {
		return -1;
	}
}
