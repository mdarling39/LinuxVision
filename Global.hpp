/*
 * Global.hpp
 *
 *  Created on: May 21, 2013
 *      Author: michaeldarling
 */

#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_

#include <iostream>

/// Detect System
#ifdef __APPLE__
#define OSX     1
#define LINUX   0
#define ARM     0
#endif /*__APPLE__*/

#ifdef __linux__
#define OSX     0
#define LINUX   1
#define ARM     0
#endif /*__linux__*/

#ifdef __arm__
#define OSX     0
#define LINUX   0
#define ARM     1
#endif /*__arm__*/


/// Macros
#define DEBUG_STDOUT			// write debug info to stdout
#define DEBUG_VIDEO				// display video debug data
//#define FOUNDBLOBS_TO_FILE	// write blob coordinates to file
//#define SAVEOFF_FRAMES		// save off frames to memory
//#define POSE_TO_FILE			// write pose estimates to file
//#define DO_KALMAN_FILTER		// implement kalman filter for object tracking TODO:  Not using Kalman fitler, remove from the code


/// Conversions
const double IN2MM = 25.4;
const double MM2IN = 1/IN2MM;
const double PI = 3.14159265359;
const double RAD2DEG = 180/PI;
const int DEG2RAD = PI/180;


/// Graphical Settings
const double AXES_LN = 10*IN2MM;	// TODO:	Move AXES_LN into a private member of the PnP class  & provide setter/getter
const int NO_LEDS = 5;		        // TODO:	Move NO_LEDS into a private member of the PnP class & provide setter/getter


void selfIdentifySystem(void)
{
std::cout << "System is:   ";
#if OSX
    std::cerr << "OSX -- no longer supported" << std::endl;
#elif LINUX
    std::cout << "LINUX" << std::endl;
#else
    std::cerr << "Unknown System!" << std::endl;
#endif
#if ARM
    std::cout << "ARM" << std::endl;
#endif
}

#endif /* GLOBAL_HPP_ */
