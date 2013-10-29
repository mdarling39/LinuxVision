/*
 * Config.h
 *
 *  Created on: Oct 11, 2013
 *      Author: michaeldarling
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "v4l2_c.h"         // for custom_v4l2_init
#include "Threshold.hpp"    // for initializeThresholdObj

/// Camera settings
#if ARM
const int DEVICE = 0;
#else
const int DEVICE = 1;
#endif
const int IM_WIDTH = 640;
const int IM_HEIGHT = 480;
const int IM_FPS = 30;
#define BUF_SZ 4

// Custom camera initialiazation function
void custom_v4l2_init(void* parm_void)
{
#define V4L2_CID_C920_ZOOMVAL    0x9A090D  //Zoom           (wide=0, telephoto=500)
#define V4L2_CID_C920_AUTOFOCUS  0x9A090C  //Autofocus      (0=OFF, 1 = ON)
#define V4L2_CID_C920_FOCUSVAL   0X9A090A  //Focus Value    (min=0, max=250)
#define V4L2_C920_FOCUS_INF      0
#define V4L2_C920_FOCUS_MACRO    250

    struct v4l2Parms* parm = (struct v4l2Parms*) parm_void;

    set_parm(parm->fd, V4L2_CID_C920_AUTOFOCUS,0);    // Turn autofocus off
    set_parm(parm->fd, V4L2_CID_C920_FOCUSVAL,V4L2_C920_FOCUS_INF);   // Use infinity focus (no macro)
    set_parm(parm->fd, V4L2_CID_C920_ZOOMVAL,0);      // Wide angle zoom
    //set_parm(parm->fd, V4L2_CID_SATURATION,128);      // Adjust Saturation (0-255)
    set_parm(parm->fd, V4L2_CID_SHARPNESS,0);         // Blur the image to get smoother contours (0-255)

}



/// Threshold object initialization function
void initializeThresholdObj(Threshold &thresh_obj)
{
    CustomBlobDetector::Params blobParams;

    // TODO:  Can look at trying to read these values in from xml/yml later
	blobParams.maxPoints = 				8;
	blobParams.maxError = 				1.8;
	blobParams.minArea = 				1;
	blobParams.maxArea = 				350;
	blobParams.minCircularity = 		0.05;
	blobParams.maxCircularity = 		1.1;
	blobParams.minInertiaRatio = 		0.05;
	blobParams.maxInertiaRatio = 		1.1;
	blobParams.minConvexity = 			0.05;
	blobParams.maxConvexity = 			1.1;
	blobParams.minThreshold = 			225;


	blobParams.targetCircularity = 		1.0;
	blobParams.targetInertiaRatio = 	1.0;
	blobParams.targetConvexity = 		1.0;
	blobParams.targetBlobColor = 		255;

	blobParams.w_Circularity = 			200;
	blobParams.w_InertiaRatio = 		0;
	blobParams.w_Convexity = 			100;
	blobParams.w_BlobColor = 			255;

	blobParams.filterByError = 			false;
	blobParams.filterByArea = 			true;
	blobParams.filterByColor = 			true;
	blobParams.filterByCircularity = 	true;
	blobParams.filterByInertia = 		true;
	blobParams.filterByConvexity = 		true;

	thresh_obj.set_params(blobParams);
}

/// Camera intrinisics and 3-D model geometry
// path to intrinsic camera properties
const std::string camDataFilename =
		"/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/C920-640x480_IntrinsicParams.yml";

// path to 3-D model geometry file
const char* modelPointsFilename =
		"/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/Glider_Geom.txt";


/// Pose estimate error tolerances
const double POSE_ERR_TOL = 0.026;				// if reprojection error is lower than this --> move on
const double SECONDARY_POSE_ERR_TOL = 0.075;	// otherwise, try re-ordering LED's and choose lowest
                                                // error that still satisfies the secondary error tolerance

/// Flight data recording
const char blobFilename[] = "blobFile.txt";		// log file name for blob detection
const char imageSavepath[] = "TestImages";		// directory to save debug frames in
const char poseFilename[] = "poseFile.txt";		// log file name for pose estimates


#endif /* CONFIG_H_ */
