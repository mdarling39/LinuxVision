/*
 * Config.h
 *
 *  Created on: Oct 11, 2013
 *      Author: michaeldarling
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "v4l2_c.h"         // for custom_v4l2_init
#include "LED_Detector.hpp"    // for definition of Threshold class
#include <unistd.h>

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
    set_parm(parm->fd, V4L2_CID_SATURATION,128);      // Adjust Saturation (0-255)
    set_parm(parm->fd, V4L2_CID_SHARPNESS,128);         // Blur the image to get smoother contours (0-255)

    //set_auto_exposure(parm->fd);
    set_manual_exposure(parm->fd, 30); // 4 to 2046
    set_parm(parm->fd, V4L2_CID_GAIN, 255); // Compensate for over/underexposure (0 to 255)



}



/// Threshold object initialization function
void initializeFeatureDetectObj(LED_Detector::Params &params)
{
    params.w_Circularity =         255;
    params.w_InertiaRatio =        255;
    params.w_Convexity =           255;
    params.w_BlobColor =           255;

    params.maxPoints =             10;
    params.threshold =             235; // Implemented
    params.minDistBetweenBlobs =   5;

    params.filterByColor =         false;
    params.targetBlobColor =       255;

    params.filterByArea =          true;  // Implemented
    params.minArea =               25;
    params.maxArea =               250;

    params.filterByCircularity =   false;  // Implemented
    params.minCircularity =        0.30;
    params.maxCircularity =        1.1;
    params.targetCircularity =     1.0;

    params.filterByInertia =       false;
    params.minInertiaRatio =       0.0;
    params.maxInertiaRatio =       0.0;
    params.targetInertiaRatio =    1.0;

    params.filterByConvexity =     false;  // Implemented
    params.minConvexity =          0.80;
    params.maxConvexity =          1.1;
    params.targetConvexity =       1.0;

    params.filterByError =         false;
    params.maxError =              2;
}

/// Camera intrinisics and 3-D model geometry
// path to intrinsic camera properties
const std::string camDataFilename =
		"/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/C920-640x480_IntrinsicParams.yml";
		//"/home/ubuntu/Vision/multithreading/Calibration/C920-640x480_IntrinsicParams.yml";


// path to 3-D model geometry file
const char* modelPointsFilename =
		"/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/Glider_Geom.txt";
		//"/home/ubuntu/Vision/multithreading/Calibration/Glider_Geom.txt";


/// Pose estimate error tolerances
const double POSE_ERR_TOL = 0.026;				// if reprojection error is lower than this --> move on
const double SECONDARY_POSE_ERR_TOL = 0.075;	// otherwise, try re-ordering LED's and choose lowest
                                                // error that still satisfies the secondary error tolerance

/// Flight data recording
char imageSavepath[] = "TestImages";		// directory to save debug frames in
unsigned int frameSkip_ms = 2000;           // Sets how often to save an image (in milliseconds)



#endif /* CONFIG_H_ */
