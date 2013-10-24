/*
 * Config.h
 *
 *  Created on: Oct 11, 2013
 *      Author: michaeldarling
 */

#ifndef CONFIG_H_
#define CONFIG_H_



int devNo = 1;		// device number of camera

const char blobFilename[] = "blobFile.txt";		// log file name for blob detection
const char imageSavepath[] = "TestImages";		// directory to save debug frames in
const char poseFilename[] = "poseFile.txt";		// log file name for pose estimates

// path to intrinsic camera properties
const std::string camDataFilename =
		"/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/C920-640x480_IntrinsicParams.yml";

// path to 3-D model geometry file
const char* modelPointsFilename =
		"/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/Glider_Geom.txt";

const double POSE_ERR_TOL = 0.026;				// if reprojection error is lower than this --> move on
const double SECONDARY_POSE_ERR_TOL = 0.075;	// otherwise, try re-ordering LED's and choose lowest
												// error that still satisfies the secondary error tolerance


#endif /* CONFIG_H_ */
