/*
 * CamObj.cpp
 *
 *  Created on: May 21, 2013
 *      Author: michaeldarling
 */

#include "CamObj.hpp"

// ============================================================================
// System specific function wrappers

#ifdef OSX
	CamObj::CamObj(){

	}

	CamObj::~CamObj(){
		CamObj::close();
	}

	bool CamObj::open(int dev){

		if (!CamObj::open_osx(dev)){
			std::cerr << "Could not open device for capturing!" << std::endl;
			return false;
		}else{
			std::cout << "Camera opened successfully" << std::endl;
			return true;
		}
	}

	void CamObj::close(){
		CamObj::close_osx();
	}

	bool CamObj::set_image_size(int desiredWidth, int desiredHeight){
		return CamObj::set_image_size_osx(desiredWidth, desiredHeight);
	}

	bool CamObj::set_framerate(int desired_fps){
		return CamObj::set_framerate_osx(desired_fps);
	}

	static void CamObj::operator>>(cv::Mat &img){
		get_frame_osx(img);
	}

#elif LINUX
	CamObj::CamObj(){

	}

	CamObj::~CamObj(){
		CamObj::close();
	}

	bool CamObj::open(int dev){

		if (!CamObj::open_linux(dev)){
			std::cerr << "Could not open device for capturing!" << std::endl;
			return false;
		}else{
			std::cout << "Camera opened successfully" << std::endl;
			return true;
		}
	}

	void CamObj::close(){
		CamObj::close_linux();
	}

	bool CamObj::set_image_size(int desiredWidth, int desiredHeight){
		return CamObj::set_image_size_linux(desiredWidth, desiredHeight);
	}

	bool CamObj::set_framerate(int desired_fps){
		return CamObj::set_framerate_linux(desired_fps);
	}

	void CamObj::operator>>(cv::Mat &img){
		get_frame_linux(img);
	}

#else
#error "Could not detect System!"
#endif
// ============================================================================




// System-specific functions
#ifdef OSX
	bool CamObj::open_osx(int dev){

		// open the device
		cap.open(dev);

		// check if opened successfully
		if (cap.isOpened()){
			return true;
		}else{
			return false;
		}
	}

	void CamObj::close_osx(){
		cap.release();
	}

	bool CamObj::set_image_size_osx(int desiredWidth, int desiredHeight){

		// set properties
		cap.set(CV_CAP_PROP_FRAME_WIDTH,desiredWidth);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT,desiredHeight);

		/* Doesn't work anyways
		// verify that the properties were set correctly
		int w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		int h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

		// return true on success
		if ((w==desiredWidth) & (h==desiredHeight)){
			return true;
		}else{
			std::cout << "WARNING:  Could not set desired frame size.  Actual size: "
					<< w << "x" << h << std::endl;
			return false;
		}
		*/
		return true;
	}

	bool CamObj::set_framerate_osx(int desired_fps){

		// set framerate
		cap.set(CV_CAP_PROP_FPS,desired_fps);

		/* Doesn't work anyways
		//verify that the framerate was set correclty
		int fps = cap.get(CV_CAP_PROP_FPS);

		// return true on success
		if (fps==desired_fps){
			return true;
		}else{
			std::cout << "WARNING:  Could not set desired framerate.  Actual fps: "
					<< fps << std::endl;
			return false;
		}
		*/
		return true;
	}

	void CamObj::get_frame_osx(cv::Mat &img){
		cap.read(img);
	}

#elif LINUX
	bool CamObj::open_linux(int dev){

		char devName[15];
		sprintf(devName,"/dev/video%d",dev);
		cap.set_dev_name(devName);
		cap.set_pix_fmt(V4L2_PIX_FMT_MJPEG);
		cap.customInit = true;

		// open the device
		cap.open_device();
		return true;

	}

	void CamObj::close_linux(){
		cap.close_device();
	}

	bool CamObj::set_image_size_linux(int desiredWidth, int desiredHeight){

	    cap.set_width(desiredWidth);                     // set frame dims.
	    cap.set_height(desiredHeight);
	    return true;
	}

	bool CamObj::set_framerate_linux(int desired_fps){

		// set framerate
		cap.set_fps(desired_fps);

	}

void CamObj::get_frame_linux(cv::Mat &img){

		cap.grab_frame(img);

	}
#endif
