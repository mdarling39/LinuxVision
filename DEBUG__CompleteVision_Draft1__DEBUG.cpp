



// SYSTEM INCLUDES
#include <iostream>
#include <iterator>



// USER INCLUDES
#include "Global.hpp"
#include "CamObj.hpp"
#include "FPSCounter.hpp"
#include "Threshold.hpp"
#include "PnPObj.hpp"

#ifdef LINUX
//#include "OCVCapture.h"
#endif /*LINUX*/

// #ifdef FOUNDBLOBS_TO_FILE
#include<stdio.h>
FILE *pFile;

// #ifdef SAVEOFF_FRAMES
#include <sys/types.h>
#include <sys/stat.h>
unsigned int frameCount = 0;
unsigned int frameSkip = 30;



// GLOBALS
int devNo = 1;		// device number of camera

const char blobFilename[] = "blobFile.txt";		// log file name for blob detection
const char imageSavepath[] = "TestImages";		// directory to save debug frames in

// path to intrinsic camera properties
const std::string camDataFilename =
		"/Users/michaeldarling/Dropbox/Thesis/OpenCV/Calibration/Zoomed In/PS3Eye_out_camera_data.yml";
// path to 3-D model geometry file
const char* modelPointsFilename =
		"/Users/michaeldarling/Dropbox/Thesis/OpenCV/Calibration/Zoomed In/LEDPos copy2.txt";

// const int NO_LEDS    // moved to Global.hpp
const double POSE_ERR_TOL = 0.026;			// if reprojection error is lower than this --> move on
const double SECONDARY_POSE_ERR_TOL = 0.055;	// otherwise, try re-ordering LED's and choose lowest
// error that still satisfies the secondary error tolerance


// HELPER FUNCTION DEFINITIONS
void writeout_blobfile_header(const char* blobFilename, const CustomBlobDetector::Params blobParams);
void setup_images_dir(const char* imageDirname, const cv::Mat &frame);


int main() {

	/////////////////////////////////////////////////////////////////////////////////
	// Display system information

#if OSX
	std::cout << "Detected OS:  OSX" << std::endl;
#elif LINUX
	std::cout << "Detected OS:  LINUX" << std::endl;
#else
	std::cerr << "Unknown System!" << std::endl;
#endif


	/////////////////////////////////////////////////////////////////////////////////
	// Initialize camera, set parameters, and grab some "throwaway" frames
//
//	std::cout << "Initializing camera" << std::endl;
//
//	cv::Mat frame;
//	CamObj cap;
//#ifdef OSX
//	// have to open camera first if OSX
//	cap.open(devNo);
//	cap.set_image_size(640,480);
//	cap.set_framerate(30);
//#elif LINUX
//	// have to set parameters first if Linux
//	cap.set_image_size(640,480);
//	cap.set_framerate(30);
//	cap.open(devNo);
//#endif
//
//
//
//
//	std::cout << "   grabbing 'throwaway' images" << std::endl;
//	cv::namedWindow("Throwaway");
//
//	for (int i=0 ; i<120; i++)
//	{
//		cap >> frame;
//
//#ifdef DEBUG_VIDEO
//		cv::imshow("Throwaway",frame);
//		cv::waitKey(1);
//#endif /*DEBUG_VIDEO*/
//	}
//
//	cv::destroyAllWindows();
//
//	std::cout << "Camera initialized" << std::endl;
//
//	//////////////////////////////////////////////////////////////////////////////////
//	// Initialize Threshold object
//
//	CustomBlobDetector::Params blobParams;
//
//	// TODO:  Can look at trying to read these values in from xml/yml later
//	blobParams.maxPoints = 				10;
//	blobParams.maxError = 				0.30;
//	blobParams.minArea = 				1;
//	blobParams.maxArea = 				200;
//	blobParams.minCircularity = 		0.05;
//	blobParams.maxCircularity = 		1.1;
//	blobParams.minInertiaRatio = 		0.05;
//	blobParams.maxInertiaRatio = 		1.1;
//	blobParams.minConvexity = 			0.05;
//	blobParams.maxConvexity = 			1.1;
//	blobParams.minThreshold = 			250;
//
//
//	blobParams.targetCircularity = 		1.0;
//	blobParams.targetInertiaRatio = 	1.0;
//	blobParams.targetConvexity = 		1.0;
//	blobParams.targetBlobColor = 		255;
//
//	blobParams.w_Circularity = 			255;
//	blobParams.w_InertiaRatio = 		0;
//	blobParams.w_Convexity = 			0;
//	blobParams.w_BlobColor = 			255;
//
//	blobParams.filterByError = 			false;
//	blobParams.filterByArea = 			true;
//	blobParams.filterByColor = 			true;
//	blobParams.filterByCircularity = 	true;
//	blobParams.filterByInertia = 		true;
//	blobParams.filterByConvexity = 		true;
//
//	// create Threshold object
//	Threshold thresh;
//	thresh.set_params(blobParams);
//
//	std::cout << "blobParams set" << std::endl;
//
//	/////////////////////////////////////////////////////////////////////////////////
//	// Setup PnP object and associated parameters
//	std::cout << "Reading intrinsic camera properties and 3-D model geometry" << std::endl;
//
//	PnPObj PnP;
//	PnP.setCamProps(camDataFilename);
//	PnP.setModelPoints(modelPointsFilename);
//
//	std::cout << "Intrinsic camera properties and 3-D geometry read-in" << std::endl;
//
//	/////////////////////////////////////////////////////////////////////////////////
//	// Setup before main loop starts
//
//	// initialize fps clock
//	FPSCounter FPSclk(15);	// Use 15-frame moving average
//
//#ifdef FOUNDBLOBS_TO_FILE
//	// begin blobFile and writeout header
//	writeout_blobfile_header(blobFilename,blobParams);
//#endif /*FOUNDBLOBS_TO_FILE*/
//
//#ifdef SAVEOFF_FRAMES
//	// clear the folder and write a reference RGB frame
//	setup_images_dir(imageSavepath,frame);
//#endif /*SAVEOFF_FRAMES*/
//
//#ifdef DEBUG_VIDEO
//	// Much faster if we create a named window here
//	cv::namedWindow("DEBUG_VIDEO");
//#endif /*DEBUG_VIDEO*/
//
//
//	std::cout << "Entering main loop" << std::endl;
//	/////////////////////////////////////////////////////////////////////////////////
//	// BEGIN MAIN LOOP
//	/////////////////////////////////////////////////////////////////////////////////

//	do
//	{

//		// capture a frame from camera
//		cap >> frame;
//
//		// peel off Red channel
//		cv::Mat R(frame.size(),CV_8UC1);
//		int mixch[] = {2,0};
//		cv::mixChannels(&frame,1,&R,1,mixch,1);
//
//
//		// detect blobs in image
//#ifdef FOUNDBLOBS_TO_FILE
//		// if we want to write results to a file, we need to reopen the logfile
//		pFile = fopen(blobFilename,"a");
//		thresh.set_image(R);
//		thresh.detect_blobs();
//		std::vector<cv::Point2f> imagePoints = thresh.get_points();
//		fclose(pFile);  // we need to re-close the log file to protect against
//		// data loss if the program is forced to terminate
//#else
//		thresh.set_image(R);
//		thresh.detect_blobs();
//		std::vector<cv::Point2f> imagePoints = thresh.get_points();
//#endif




		/*
		// estimate pose  (if we have at least as many blobs as LEDs)
		// TODO:  Add ability to swap out/reorder "best" LEDs -- Clean PnP solution up!

		// TODO:  Implement in PnPObj.cpp as a "new_iteration()" member or something
		PnP.is_current = false;

		// repeat pose estimate until error tolerance is met, otherwise start over with a new frame
		std::vector<double> poseState;

		double poseErr = INFINITY;
		if (imagePoints.size() >= (unsigned int)NO_LEDS){

			// pass imagePoints to PnP object
			std::vector<cv::Point2f>::iterator sub_first = imagePoints.begin();
			PnP.setImagePoints(sub_first,sub_first + NO_LEDS);

			//TODO: Add a n_iter counter to keep track of the number of attempts at finding a suitable pose
			for (int i=0; i<=2; i++){	//TODO:  Currently, only rearranging LED correlation twice then giving up
				PnP.correlatePoints(i);
				PnP.solve();
				double thisErr = PnP.getScaledReprojError();

				if (thisErr < POSE_ERR_TOL){ // Meets primary tolerance -- return with this state immediately
					poseErr = thisErr;
					poseState = PnP.getState();
					break;
				} else if (thisErr < SECONDARY_POSE_ERR_TOL){ 	// Meets secondary tolerance -- keep in memory,
																// but see if we can do better

					if(thisErr < poseErr) { // We have made an improvement from the last case
											// update with this state
						poseErr = thisErr;
						poseState = PnP.getState();
					}
				} else {
					// Pose estimate might have been totally off, revert to "reasonable" guess
					PnP.resetGuess();
				}
			}


			if (PnP.is_current) {

				// Convert units
				poseState[0] *= MM2IN;
				poseState[1] *= MM2IN;
				poseState[2] *= MM2IN;
				poseState[3] *= RAD2DEG;
				poseState[4] *= RAD2DEG;
				poseState[5] *= RAD2DEG;
			}


		}else{ // Do not have enough LEDs to compute a pose estimate

			// TODO:  Need to do something else here
		}
		*/


	PnPObj PnP;
	PnP.setCamProps(camDataFilename);
	PnP.setModelPoints(modelPointsFilename);

	std::vector<double> poseState(6);
	double poseErr = INFINITY;


	// DEBUG__ create image points __DEBUG
	std::vector<cv::Point2f> _imagePoints;
//	imagePoints.push_back(cv::Point2f(184.1783, 245.9922)); // LW
//	imagePoints.push_back(cv::Point2f(291.9149, 229.2624)); // LH
//	imagePoints.push_back(cv::Point2f(363.5597, 182.0692)); // VT
//	imagePoints.push_back(cv::Point2f(453.2947, 210.9324)); // RH
//	imagePoints.push_back(cv::Point2f(563.3046, 189.6264)); // RW

	_imagePoints.push_back(cv::Point2f(453.2947, 210.9324)); // RH
	_imagePoints.push_back(cv::Point2f(563.3046, 189.6264)); // RW
	_imagePoints.push_back(cv::Point2f(363.5597, 182.0692)); // VT
	_imagePoints.push_back(cv::Point2f(184.1783, 245.9922)); // LW
	_imagePoints.push_back(cv::Point2f(358.8500, 372.5500)); // noise
	_imagePoints.push_back(cv::Point2f(291.9149, 229.2624)); // LH



// first call
	std::vector<cv::Point2f> imagePoints = _imagePoints;

	PnP.localizeUAV(imagePoints,poseState, poseErr, 12, 0.026, 0.55);
	poseState = PnP.getState();
	poseState[0] *= MM2IN;
	poseState[1] *= MM2IN;
	poseState[2] *= MM2IN;
	poseState[3] *= RAD2DEG;
	poseState[4] *= RAD2DEG;
	poseState[5] *= RAD2DEG;

std::cout << "State:  " << poseState[0] << " "
				 << " " << poseState[1] << " "
				 << " " << poseState[2] << " "
				 << " " << poseState[3] << " "
				 << " " << poseState[4] << " "
				 << " " << poseState[5] << " " << std::endl;

std::cout << "Reproj Err " << PnP.getScaledReprojError() << std::endl;



// second call
std::vector<cv::Point2f> _imagePoints2;
_imagePoints2.push_back(cv::Point2f(360.4025,  106.1258));
_imagePoints2.push_back(cv::Point2f(285.2071,  141.0303));
_imagePoints2.push_back(cv::Point2f(439.8465,  150.1535));
_imagePoints2.push_back(cv::Point2f(188.1565,   94.5374));
_imagePoints2.push_back(cv::Point2f(563.4222,  104.9111));

	imagePoints = _imagePoints;

	PnP.localizeUAV(imagePoints,poseState, poseErr, 6, 0.01, 0.55);
	PnP.localizeUAV(imagePoints,poseState, poseErr, 6, 0.02, 0.55);
	PnP.localizeUAV(imagePoints,poseState, poseErr, 6, 0.02, 0.55);
	poseState = PnP.getState();
	poseState[0] *= MM2IN;
	poseState[1] *= MM2IN;
	poseState[2] *= MM2IN;
	poseState[3] *= RAD2DEG;
	poseState[4] *= RAD2DEG;
	poseState[5] *= RAD2DEG;

std::cout << "State:  " << poseState[0] << " "
				 << " " << poseState[1] << " "
				 << " " << poseState[2] << " "
				 << " " << poseState[3] << " "
				 << " " << poseState[4] << " "
				 << " " << poseState[5] << " " << std::endl;

std::cout << "Reproj Err " << PnP.getScaledReprojError() << std::endl;




// Manual Implementation
	imagePoints = _imagePoints;

	PnP.swapImagePoints(imagePoints,imagePoints,1);
	PnP.setImagePoints(imagePoints.begin(),imagePoints.end());
	PnP.correlatePoints(0);

	PnP.solve();

	poseState = PnP.getState();
	poseState[0] *= MM2IN;
	poseState[1] *= MM2IN;
	poseState[2] *= MM2IN;
	poseState[3] *= RAD2DEG;
	poseState[4] *= RAD2DEG;
	poseState[5] *= RAD2DEG;

std::cout << "State:  " << poseState[0] << " "
				 << " " << poseState[1] << " "
				 << " " << poseState[2] << " "
				 << " " << poseState[3] << " "
				 << " " << poseState[4] << " "
				 << " " << poseState[5] << " " << std::endl;

std::cout << "Reproj Err " << PnP.getScaledReprojError() << std::endl;







//		int poseIters = PnP.localizeUAV(imagePoints, poseState, poseErr, 6, POSE_ERR_TOL, SECONDARY_POSE_ERR_TOL);
//		PnP.is_current = (poseIters > 0) ? true : false;




		/* ====================== DEBUG INFO ====================== */

//#if defined(DEBUG_VIDEO) || defined(SAVEOFF_FRAMES)
//		// print blobs on image (green)
//		thresh.createBlobsImage(frame,cv::Scalar(0,255,0));
//
//
//		// print the 5 "most probable" blobs on image (blue)
//		if (imagePoints.size() > 0) {
//			for (int i = 0; i < NO_LEDS; i++) {
//				cv::circle(frame,imagePoints[i], 5, cv::Scalar(255,0,0), 3);
//			}
//		}
//
//		PnP.drawOverFrame(frame,frame);
//
//#endif /*DEBUG_VIDEO -or- SAVEOFF_FRAMES*/
//
//#ifdef SAVEOFF_FRAMES
//		frameCount++;
//		if (frameCount % frameSkip == 0) {
//
//			// add a frame number label to image
//			std::stringstream frameNoStr;
//			frameNoStr << "Frame # " << frameCount;
//			cv::putText(frame,frameNoStr.str(),cv::Point2f(20,20),
//					FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255));
//
//
//			// save off the file
//			frameNoStr.str(""); // clear the string
//			frameNoStr << imageSavepath << "/frame_" << frameCount << ".jpg";
//			cv::imwrite(frameNoStr.str(), frame);
//
//		}
//#endif /*SAVEOFF_FRAMES*/
//
//#ifdef DEBUG_VIDEO
//		cv::imshow("DEBUG_VIDEO",frame);
//		cv::waitKey(1);
//#endif /*DEBUG_VIDEO*/
//
//#ifdef DEBUG_STDOUT
//		// display the results of the blob finding
//		std::cout << "Number of blobs found -- " << imagePoints.size() << '\n';
//		//std::cout << "Image Points: " << imagePoints << std::endl;
//
//		// Print state to standard output
//		std::cout << "Estimated Pose:  " << "[";
//		if (PnP.is_current) {
//		std::copy(poseState.begin(), poseState.end()-1, std::ostream_iterator<double>(std::cout, ", "));
//		std::cout << (poseState.back());
//		std::cout <<  "]\n";
//		std::cout << "   in " << poseIters << " iterations\n";
//		std::cout << "   error: " << poseErr << "\n";
//		} else{
//			std::cout << "  --- Could not solve PnP ---  ";
//			std::cout <<  "]\n";
//		}
//
//
//		// compute and display the framerate (moving average)
//		std::cout << "Average FPS:  " << FPSclk.fps() << "\n";
//		std::cout << std::endl;
//#endif
//
//	}
//	while (true);  // TODO: Modify loop to look for keypress to terminate program
//
//	/////////////////////////////////////////////////////////////////////////////////
//	// END MAIN LOOP
//	/////////////////////////////////////////////////////////////////////////////////
//
//	std::cout << "Main loop completed successfully" << std::endl;


	return 0;

}



void writeout_blobfile_header(const char* blobFilename,const CustomBlobDetector::Params blobParams){

	std::cout << "   creating a logfile to hold blob finding data" << std::endl;

	// open file for printing to
	pFile = fopen(blobFilename,"w");

	fprintf(pFile,"%15s%15s%15s%15s%15s%15s%15s\n","weights:","","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(pFile,"%15s%15s%15d%15d%15d%15d%15s\n","(0-255)","",blobParams.w_Circularity,blobParams.w_InertiaRatio,
			blobParams.w_Convexity,blobParams.w_BlobColor,"--");

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","FilterBy:","error","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(pFile,"%15s%15d%15d%15d%15d%15d%15d\n","",blobParams.filterByError,blobParams.filterByCircularity,blobParams.filterByInertia,
			blobParams.filterByConvexity,blobParams.filterByColor,blobParams.filterByArea);

	fprintf(pFile,"\n%15s%15d%15.3f%15.3f%15.3f%15.3f%15.3f\n","min",0,blobParams.minCircularity,blobParams.minInertiaRatio,
			blobParams.minConvexity,blobParams.minThreshold,blobParams.minArea);
	fprintf(pFile,"%15s%15.3f%15.3f%15.3f%15.3f%15.3f%15.3f\n","max",blobParams.maxError,blobParams.maxCircularity,blobParams.maxInertiaRatio,
			blobParams.maxConvexity,255.0,blobParams.maxArea);

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","Targets:","","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(pFile,"%15s%15s%15.3f%15.3f%15.3f%15d%15s\n","","",blobParams.targetCircularity,blobParams.targetInertiaRatio,
			blobParams.targetConvexity,blobParams.targetBlobColor,"--");

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s%15s%15s\n","blobNo","TotalError","circularity","inertiaRatio",
			"convexity","blobColor","area","imageXpt","imageYpt");
	fclose(pFile);
}

void setup_images_dir(const char* imageDirname,const cv::Mat &frame){

	std::cout << "   setting up directory for saved frames" << std::endl;

	// clear the TestImages folder
	char str[50];
	sprintf(str,"rm -r %s",imageDirname);
	system(str);
	if (mkdir(imageDirname, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) {
		std::cout << "   directory for saving frames successfully cleared" << std::endl;
	}

	// get an RGB frame for reference
	cv::imwrite("TestImages/frame_0.jpg", frame);
}
