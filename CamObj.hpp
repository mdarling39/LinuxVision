/*
 * CamObj.hpp
 *
 *  Created on: May 21, 2013
 *      Author: michaeldarling
 */

#ifndef CAMOBJ_HPP_
#define CAMOBJ_HPP_

// Wrapper for camera capture object.  Either uses OpenCV's VideoCapture object (if Mac or Windows)
// or uses Martin Fox's OCVCapture object.
//
// TODO:  Possibly add a third system type to handle BeagleBone (if special handling needed)

// Detect System
#ifdef __APPLE__
#define OSX 1
#endif /*__APPLE__*/

#ifdef __linux__
#define LINUX 1
#endif /*__linux__*/



#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if OSX
#include <opencv2/video/video.hpp>
#endif
#if LINUX
#include "v4l2Cap.h"
#endif




class CamObj {

	// System specific member functions and variables
#if OSX
	typedef cv::VideoCapture cap_t;
	cap_t cap;
	bool open_osx(int dev_osx);
	void close_osx();
	bool set_image_size_osx(int desiredWidth, int desiredHeight);
	bool set_framerate_osx(int desired_fps);
	void get_frame_osx(cv::Mat &img);

#elif LINUX
	typedef v4l2Cap cap_t;
	cap_t cap;
	bool open_linux(int dev_linux);
	void close_linux();
	bool set_image_size_linux(int desiredWidth, int desiredHeight);
	bool set_framerate_linux(int desired_fps);
	void get_frame_linux(cv::Mat &img);
#endif

public:

	// default constructor/destructor
	CamObj();
	~CamObj();

	void close();
	bool open(int dev);

	// setters  (return 1 if success, 0 otherwise)
	bool set_image_size(int desiredWidth, int desiredHeight);
	bool set_framerate(int desired_fps);

	// get image
	void operator>>(cv::Mat &img);


};





#endif /* CAMOBJ_HPP_ */
