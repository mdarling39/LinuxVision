/*
 * FPSCounter.h
 *
 *  Created on: Apr 6, 2013
 *      Author: michaeldarling
 */

#ifndef FPSCOUNTER_H_
#define FPSCOUNTER_H_


#include <sys/time.h>
#include <stddef.h>
#include <iostream>
#include "Global.hpp"

class FPSCounter {

	double mySec, myUsec, init_secs, init_usecs;
	int* dtBuffer;
	int MAVG_n;
	timeval now;

public:

	int elapsedTime;

	// default constructors
	FPSCounter();
	FPSCounter(int init_fps);

	// default destructor
	~FPSCounter();

	// reset timer
	void resetClk();

	// get the elapsed time (milliseconds)
	double t_elapsed();

	// compute the moving average of the framerate
	double fps();

};


#endif /* FPSCOUNTER_H_ */
