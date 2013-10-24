/*
 * FPSCounter.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: michaeldarling
 */

#include "FPSCounter.hpp"


// default constructor
FPSCounter::FPSCounter(int n) {
	MAVG_n = n;
	mySec = 0;
	myUsec = 0;
	init_secs = 0;
	init_usecs = 0;
	elapsedTime = 0;

	// save zeros to buffer
	dtBuffer = new int [MAVG_n];
	for (int i=0; i<MAVG_n; i++) {
		dtBuffer[i] = 0;
	}

	FPSCounter::resetClk();
}

// default destructor
FPSCounter::~FPSCounter() {
	delete [] dtBuffer;
}

// reset timer
void FPSCounter::resetClk(){
	gettimeofday(&now, NULL);
	init_secs = now.tv_sec;
	init_usecs = now.tv_usec;
}

// get the elapsed time (milliseconds)
double FPSCounter::t_elapsed() {
	gettimeofday(&now, NULL);
	mySec = now.tv_sec - init_secs;
	myUsec = now.tv_usec - init_usecs;
	return elapsedTime = (int) (mySec*1000) + (myUsec/1000);
}

// get fps
double FPSCounter::fps() {

	// shift the buffer
	for (int i=0;i<(MAVG_n-1);i++) {
		dtBuffer[i] = dtBuffer[i+1];
	}

	// fill the most recent element with the elapsed time since last frame
	elapsedTime = FPSCounter::t_elapsed();
	dtBuffer[MAVG_n-1] = elapsedTime;

	// rest the timer
	FPSCounter::resetClk();

	// complete the MAVG calculation by getting the last value and averaging
	double tmp = 0;
	for (int i=0; i<MAVG_n; i++) tmp += dtBuffer[i];
	tmp = MAVG_n/(tmp/1000);

	return tmp;
}
