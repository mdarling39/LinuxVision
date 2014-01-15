/*
 * Config.h
 *
 *  Created on: Oct 11, 2013
 *      Author: michaeldarling
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "v4l2_c.h"            // for custom_v4l2_init
#include "LED_Detector.hpp"    // for definition of Threshold class
#include "ThresholdedKF.hpp"   // for thresholded Kalman filter
#include <unistd.h>

#define OUTDOOR  //OUTDOOR //INDOOR // AUTO


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

// Simulation FPS (can slow faster computers to emulate embedded computer speed)
int simulated_fps = 20; // Slow down to the desired framerate (to run closer to embedded system)

// Custom camera initialiazation function
void custom_v4l2_init(void* parm_void)
{
#define V4L2_CID_C920_ZOOMVAL    0x9A090D  //Zoom           (wide=0, telephoto=500)
#define V4L2_CID_C920_AUTOFOCUS  0x9A090C  //Autofocus      (0=OFF, 1 = ON)
#define V4L2_CID_C920_FOCUSVAL   0X9A090A  //Focus Value    (min=0, max=250)
#define V4L2_C920_FOCUS_INF      0
#define V4L2_C920_FOCUS_MACRO    250

    struct v4l2Parms* parm = (struct v4l2Parms*) parm_void;

    set_parm(parm->fd, V4L2_CID_C920_AUTOFOCUS,0);      // Turn autofocus off
    set_parm(parm->fd, V4L2_CID_C920_FOCUSVAL,V4L2_C920_FOCUS_INF);   // Use infinity focus (no macro)
    set_parm(parm->fd, V4L2_CID_C920_ZOOMVAL,107);      // Zoom (trying to match PS3 Eye)
    //set_parm(parm->fd, V4L2_CID_C920_ZOOMVAL,150);
    set_parm(parm->fd, V4L2_CID_SATURATION,170);        // Adjust Saturation (0-255)
    set_parm(parm->fd, V4L2_CID_SHARPNESS,128);         // Blur the image to get smoother contours (0-255)

#ifdef INDOOR
    set_manual_exposure(parm->fd, 20);  // Indoor exposure
    set_parm(parm->fd, V4L2_CID_GAIN, 255); // Indoor gain
#endif

#ifdef OUTDOOR
    set_manual_exposure(parm->fd, 3); // Outdoor exposure (4 to 2046)
    set_parm(parm->fd, V4L2_CID_GAIN, 15); // Outdoor gain
#endif

#ifdef AUTO
    set_auto_exposure(parm->fd);
#endif

    set_parm(parm->fd, V4L2_CID_BRIGHTNESS, 128);
    set_parm(parm->fd, V4L2_CID_CONTRAST, 128);
}



/// Threshold object initialization function
void initializeFeatureDetectObj(LED_Detector::Params &params)
{

#ifdef INDOOR
    /// Good indoor variables
    params.threshold =             180;
    params.maxBlobs =              8;

    params.sortByColor =           true;
    params.targetColor =           330;   // Red-ish
    params.filterByColor =         true;
    params.maxColor =              75;

    params.filterByArea =          true;
    params.minArea =               2;
    params.maxArea =               800;

    params.filterByCircularity =   false;
    params.minCircularity =        0.4;
    params.maxCircularity =        1.1;

    params.filterByAspectRatio =   false;
    params.minAspectRatio =        0.4;
    params.maxAspectRatio =        1.1;

    params.localRadius =           20;
#endif

#ifdef OUTDOOR
    /// Good outdoor vairables
    params.threshold =             150;
    params.maxBlobs =              8;

    params.sortByColor =           true;
    params.targetColor =           320;   // Red-ish
    params.filterByColor =         true;
    params.maxColor =              50;

    params.filterByArea =          true;
    params.minArea =               2;
    params.maxArea =               800;

    params.filterByCircularity =   false;
    params.minCircularity =        0.4;
    params.maxCircularity =        1.1;

    params.filterByAspectRatio =   false;
    params.minAspectRatio =        0.4;
    params.maxAspectRatio =        1.1;

    params.localRadius =           20;
#endif
}


/// Kalman filter parameters
void configThresholdedKF(ThresholdedKF::param_t &KF_parms)
{
    KF_parms.A = 1;
    KF_parms.C = 1;
    KF_parms.Q = 1e-2;
    KF_parms.R = 1e-4;
    KF_parms.P0 = 0.01;
    KF_parms.KF_thresh = 1e6;
    KF_parms.timeout = 300;
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
    "/home/ubuntu/Vision/Calibration/C920-640x480_IntrinsicParams.yml";
#else
    "/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/C920-640x480_IntrinsicParams.yml";
#endif


// path to 3-D model geometry file
const char* modelPointsFilename =
#if ARM
    "/home/ubuntu/Vision/Calibration/Glider_Geom.txt";
#else
    "/home/mdarling/Desktop/CompleteVision_MAIN/Calibration/Penguin_Geom.txt";
#endif


/// Pose estimate error tolerances
const double POSE_ERR_TOL = 0.05;				// if reprojection error is lower than this --> move on
const double SECONDARY_POSE_ERR_TOL = 0.25;	// otherwise, try re-ordering LED's and choose lowest
                                                // error that still satisfies the secondary error tolerance

/// Flight data recording
char imageSavepath[] =
#if ARM
    "/media/ubuntu/microSD/TestImages";
#else
    "TestImages";		// directory to save debug frames in
#endif
unsigned int frameSkip_ms = 1000;  // Sets how often to save an image (in milliseconds)


#endif /* CONFIG_H_ */
