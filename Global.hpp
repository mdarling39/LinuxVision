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
#define POSE_TO_CONSOLE             1      // write standard info to console
#define DEBUG_STDOUT	            1		// write debug info to stdout
#define DEBUG_VIDEO		            1       // display video debug data
#define FOUNDBLOBS_TO_FILE	        1       // write blob coordinates to file
//#define POSE_TO_FILE			    1       // write pose estimates to file
//#define SAVEOFF_FRAMES		        1       // save off frames to memory
//#define DO_KALMAN_FILTER		    1       // implement kalman filter for object tracking -- (not functional)


/// Conversions
const double IN2MM = 25.4;
const double MM2IN = 1/IN2MM;
const double PI = 3.14159265359;
const double RAD2DEG = 180/PI;
const int DEG2RAD = PI/180;


/// Graphical Settings
const double AXES_LN = 10*IN2MM;	// TODO:	Move AXES_LN into a private member of the PnP class  & provide setter/getter
const int NO_LEDS = 5;		        // TODO:	Move NO_LEDS into a private member of the PnP class & provide setter/getter



/// Flight data recording
char blobFilename[] = "blobFile.txt";		// log file name for blob detection
char imageSavepath[] = "TestImages";		// directory to save debug frames in
char poseFilename[] = "poseFile.txt";		// log file name for pose estimates

/// Used for blobfile
#include<stdio.h>
FILE *bFile;

/// Used for posefile
FILE *pFile;

#endif /* GLOBAL_HPP_ */
