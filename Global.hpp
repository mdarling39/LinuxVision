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
#ifdef __arm__
#define OSX     0
#define LINUX   0
#define ARM     1
#endif

#if (defined __linux__ && !defined __arm__)
#define OSX     0
#define LINUX   1
#define ARM     0
#endif /*__arm__ OR __linux__*/

#ifdef __APPLE__
#define OSX     1
#define LINUX   0
#define ARM     0
#endif /*__APPLE__*/





/// Macros (uncomment to disable)
#define POSE_TO_CONSOLE             1       // write standard info to console
#define DEBUG_STDOUT	            1		// write debug info to stdout
#define DEBUG_VIDEO		            1       // display video debug data
#define SAVEOFF_FRAMES		        1       // save off frames to memory
//#define EMPLOY_KF                   1       // Kalman filter the estimated state
#define SAVE_KF_DATA                1       // save inputs and outputs of KF data to KF_Debug.txt
#define LOG_VISION_DATA             1       //save off a log file

// SAVE_KF_DATA may not be defined if we are not Kalman Filtering
#ifndef EMPLOY_KF
#undef SAVE_KF_DATA
#endif //EMPLOY_KF

#if ARM  // never try and display video on ARM system
#undef DEBUG_VIDEO
#endif


/// Conversions
const double IN2MM = 25.4;
const double MM2IN = 1/IN2MM;
const double PI = 3.14159265359;
const double RAD2DEG = 180/PI;
const int DEG2RAD = PI/180;


/// Graphical Settings
const double AXES_LN = 10*IN2MM;	// TODO:	Move AXES_LN into a private member of the PnP class  & provide setter/getter
const int NO_LEDS = 5;		        // TODO:	Move NO_LEDS into a private member of the PnP class & provide setter/getter

#endif /* GLOBAL_HPP_ */
