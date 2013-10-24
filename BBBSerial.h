/*
 * BBBSerial.h
 *
 *  Created on: Oct 7, 2013
 *      Author: michaeldarling
 */

#ifndef BBBSERIAL_H_
#define BBBSERIAL_H_

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>
#include <vector>

#define BBBSERIAL_BAUD	  B38400

typedef unsigned char uint8_t;

class BBBSerial{
private:
	int fd;						// file handle
	int rd; 					// read character count
	unsigned int read_timeout;	// timeout on read

	// header message, initial checksum, payload length
	uint8_t hdr[4], chk_hdr, payload_len;



public:

	BBBSerial();
	~BBBSerial();
	bool readBytes(uint8_t&);
	void writeBytes(const uint8_t&);
	int writeData(const std::vector<double>&);	// Pass a vector of state info
	int checkRequest();

	// TODO:
	// Need to Send Header Message: 'D' 'A' 'T' 'A'
	// Send PAYLOAD length as a byte
	// Typecast payload to single-precision floats (probably save as a float data[6] type array)
	// Use assert(sizeof(float) == 4) to make sure that a float is 4 bytes
	// Use a union to be able to access payload as an array of floats or an array of uint8_t.
	// Compute a checksum by doing a bitwise OR (^) on all bytes before the checksum in sequence.
	// Append all bytes in order and send across serial

};


#endif /* BBBSERIAL_H_ */
