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
    set_parm(parm->fd, V4L2_CID_SATURATION,170);      // Adjust Saturation (0-255)
    set_parm(parm->fd, V4L2_CID_SHARPNESS,128);         // Blur the image to get smoother contours (0-255)

    //set_auto_exposure(parm->fd);

    set_manual_exposure(parm->fd, 200);  // Indoor exposure
    set_parm(parm->fd, V4L2_CID_GAIN, 255); // Indoor gain

    //set_manual_exposure(parm->fd, 3); // Outdoor exposure (4 to 2046)
    //set_parm(parm->fd, V4L2_CID_GAIN, 20); // Outdoor gain

    set_parm(parm->fd, V4L2_CID_BRIGHTNESS, 128);
    set_parm(parm->fd, V4L2_CID_CONTRAST, 128);

}



/// Threshold object initialization function
void initializeFeatureDetectObj(LED_Detector::Params &params)
{
    /// Good indoor variables
    params.threshold =             200;
    params.maxBlobs =              8;

    params.sortByColor =           true;
    params.targetColor =           320;   // Red-ish
    params.filterByColor =         true;
    params.maxColor =              90;

    params.filterByArea =          true;
    params.minArea =               3;
    params.maxArea =               300;

    params.filterByCircularity =   false;
    params.minCircularity =        0.4;
    params.maxCircularity =        1.1;

    params.filterByAspectRatio =   false;
    params.minAspectRatio =        0.3;
    params.maxAspectRatio =        1.1;

    params.localRadius =           10;

    /// Good outdoor vairables
    /*
    params.threshold =             200;
    params.maxBlobs =              8;

    params.sortByColor =           true;
    params.targetColor =           320;   // Red-ish
    params.filterByColor =         true;
    params.maxColor =              75;

    params.filterByArea =          true;
    params.minArea =               3;
    params.maxArea =               500;

    params.filterByCircularity =   false;
    params.minCircularity =        0.4;
    params.maxCircularity =        1.1;

    params.filterByAspectRatio =   false;
    params.minAspectRatio =        0.4;
    params.maxAspectRatio =        1.1;

    params.localRadius =           10;
    */
}


bool checkSanity(const vector<double> &poseState)
{
    // Should be behind leader
    if (poseState[0] < 0)
        return -1;

    // All angles should be less than +/- 90 degrees
    for (int idx=3; idx<=5; idx++)
    {
        if (poseState[idx] < -90 || poseState[idx] > 90)
            return -1;
    }

    return 1;
}

/// Camera intrinisics and 3-D model geometry
// path to intrinsic camera properties
const std::string camDataFilename =
#if ARM
    "/home/ubuntu/Vision/multithreading/Calibration/C920-640x480_IntrinsicParams.yml";
#else
    "/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/C920-640x480_IntrinsicParams.yml";
#endif


// path to 3-D model geometry file
const char* modelPointsFilename =
#if ARM
    "/home/ubuntu/Vision/multithreading/Calibration/Glider_Geom.txt";
#else
    "/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/Glider_Geom.txt";
#endif


/// Pose estimate error tolerances
const double POSE_ERR_TOL = 0.030;				// if reprojection error is lower than this --> move on
const double SECONDARY_POSE_ERR_TOL = 0.050;	// otherwise, try re-ordering LED's and choose lowest
                                                // error that still satisfies the secondary error tolerance

/// Flight data recording
char imageSavepath[] = "TestImages";		// directory to save debug frames in
unsigned int frameSkip_ms = 2000;           // Sets how often to save an image (in milliseconds)



#endif /* CONFIG_H_ */
