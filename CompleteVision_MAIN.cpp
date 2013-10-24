/*
 * TODO: Implement frame saving to uSD card
 * TODO: Clean-up code as needed
 *
 */

// SYSTEM INCLUDES
#include <iostream>
#include <iterator>

// USER INCLUDES
#include "Global.hpp"
#include "Config.hpp"
#include "CamObj.hpp"
#include "FPSCounter.hpp"
#include "Threshold.hpp"
#include "PnPObj.hpp"
#include "BBBSerial.h"

// #ifdef FOUNDBLOBS_TO_FILE
#include<stdio.h>
FILE *bFile;
bool modeOn = false;

// #ifdef POSE_TO_FILE
FILE *pFile;

// #ifdef SAVEOFF_FRAMES
#include <sys/types.h>
#include <sys/stat.h>
unsigned int frameCount = 0;
unsigned int frameSkip = 30;


// HELPER FUNCTION DEFINITIONS
void writeout_blobfile_header(const char* blobFilename, const CustomBlobDetector::Params blobParams);
void writeout_posefile_header(const char* poseFilename);
void setup_images_dir(const char* imageDirname, const cv::Mat &frame);

// Global Declarations
cv::Mat frame(cv::Size(640,480),CV_8UC3);
cv::Mat R(frame.size(),CV_8UC1);
Threshold thresh;
PnPObj PnP;



int main_POOP() {

	/////////////////////////////////////////////////////////////////////////////////
	// Display system information

#if OSX
	std::cout << "Detected OS:  OSX" << std::endl;
#elif LINUX
	std::cout << "Detected OS:  LINUX" << std::endl;
#else
	std::cerr << "Unknown System!" << std::endl;
#endif
#if ARM
	std::cout << "Detected Beaglebone" << std::endl;
#endif


	/////////////////////////////////////////////////////////////////////////////////
	// Initialize camera, set parameters, and grab some "throwaway" frames
	std::cout << "Initializing camera" << std::endl;

	 //cv::Mat frame(cv::Size(640,480),CV_8UC3);
	 CamObj cap;
#ifdef OSX
	// have to open camera first if OSX
	if(!cap.open(devNo)) return -1;
	cap.set_image_size(640,480);
	cap.set_framerate(30);
#elif LINUX
	// have to set parameters first if Linux
	cap.set_image_size(640,480);
	cap.set_framerate(30);
	if(!cap.open(devNo)) return -1;
#endif

	cv::destroyAllWindows();
	std::cout << "Camera initialized" << std::endl;

	//////////////////////////////////////////////////////////////////////////////////
	// Initialize Threshold object

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
	blobParams.minThreshold = 			250;


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

	// create Threshold object
	//Threshold thresh;
	thresh.set_params(blobParams);

	std::cout << "blobParams set" << std::endl;

	/////////////////////////////////////////////////////////////////////////////////
	// Setup PnP object and associated parameters
	std::cout << "Reading intrinsic camera properties and 3-D model geometry" << std::endl;

	//PnPObj PnP;
	PnP.setCamProps(camDataFilename);
	PnP.setModelPoints(modelPointsFilename);

	std::cout << "Intrinsic camera properties and 3-D geometry read-in" << std::endl;

	/////////////////////////////////////////////////////////////////////////////////
	// Setup before main loop starts

	// initialize fps clock
	 static FPSCounter FPSclk(15);	// Use 15-frame moving average

	// initialize serial ports
#if ARM
	BBBSerial Serial;
#endif

#ifdef FOUNDBLOBS_TO_FILE
	// begin blobFile and writeout header
	writeout_blobfile_header(blobFilename,blobParams);
#endif /*FOUNDBLOBS_TO_FILE*/

#ifdef POSE_TO_FILE
	// begin poseFile and writeout header
	writeout_posefile_header(poseFilename);
#endif /*POSE_TO_FILE*/

#ifdef SAVEOFF_FRAMES
	// clear the folder and write a reference RGB frame
	setup_images_dir(imageSavepath,frame);
#endif /*SAVEOFF_FRAMES*/

#ifdef DEBUG_VIDEO
	// Much faster if we create a named window here
	cv::namedWindow("DEBUG_VIDEO");
#endif /*DEBUG_VIDEO*/


	std::cout << "Entering main loop" << std::endl;
	/////////////////////////////////////////////////////////////////////////////////
	// BEGIN MAIN LOOP
	/////////////////////////////////////////////////////////////////////////////////

    int prof_i=0;
	do
	{
        if (prof_i++ == 1000)
            return 0;

		// capture a frame from camera
		cap >> frame;

		// peel off Red channel
		//cv::Mat R(frame.size(),CV_8UC1);
		int mixch[] = {2,0};
		cv::mixChannels(&frame,1,&R,1,mixch,1);


		// detect blobs in image
#ifdef FOUNDBLOBS_TO_FILE
		// if we want to write results to a file, we need to reopen the logfile
		bFile = fopen(blobFilename,"a");
		thresh.set_image(R);
		thresh.detect_blobs();
		std::vector<cv::Point2f> imagePoints = thresh.get_points();
		fclose(bFile);  // we need to re-close the log file to protect against
		// data loss if the program is forced to terminate
#else
		thresh.set_image(R);
		thresh.detect_blobs();
		std::vector<cv::Point2f> imagePoints = thresh.get_points();
#endif


		// compute pose estimate
		std::vector<double> poseState(6);
		double poseErr;

		int poseIters = PnP.localizeUAV(imagePoints, poseState, poseErr, 6, POSE_ERR_TOL, SECONDARY_POSE_ERR_TOL);

		if (poseIters > 0) {
			PnP.is_current = true;  // set flag to know if this is an updated state or not
		} else {
			PnP.is_current = false;
			//poseIters = (int)NAN;
			//imagePoints.resize(5);
			//fill(imagePoints.begin(),imagePoints.end(),cv::Point((int)NAN,(int)NAN));
		}

#if ARM
		// send pose estimate over serial
		Serial.writeData(poseState);
#endif

#ifdef POSE_TO_FILE
		// writeout pose estimate to logfile
		pFile = fopen(poseFilename,"a");
		fprintf(pFile,"%08.4f,%08.4f,%08.4f,%08.4f,%08.4f,%08.4f   "
				      "%9.6f   "
				      "%4d   "
				      "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f",
				       poseState[0],poseState[1],poseState[2],
				       poseState[3],poseState[4],poseState[5],
				       poseErr,
				       poseIters,
				       imagePoints[0].x,imagePoints[0].y,
				       imagePoints[1].x,imagePoints[1].y,
				       imagePoints[2].x,imagePoints[2].y,
				       imagePoints[3].x,imagePoints[3].y,
				       imagePoints[4].x,imagePoints[4].y);
#endif /*POSE_TO_FILE*/
#if defined(POSE_TO_FILE) && defined(DEBUG_VIDEO)

		// Detect if we want to record
		char modeSwitch = cv::waitKey(30);
		if (!modeOn && (modeSwitch == 'y')) {
			modeOn = true;
		} else if (modeOn && (modeSwitch == 'n')) {
			modeOn = false;
		}

		// Add a mode (1/0) to know if we want to record or not
		if (modeOn) {
			fprintf(pFile,"   %d", 1);
			cv::putText(frame, "RECORDING", cv::Point2f(500,20), cv::FONT_HERSHEY_PLAIN,
				1.0, cv::Scalar(0,0,255));
		} else {
			fprintf(pFile,"   %d",0);
		}


#endif /* GROUND_TEST */
#ifdef POSE_TO_FILE
		fprintf(pFile,"\n");
		fclose(pFile);
#endif /*POSE_TO_FILE*/


		/* ====================== DEBUG INFO ====================== */
#if defined(DEBUG_VIDEO) || defined(SAVEOFF_FRAMES)
		// print blobs on image (green)
		thresh.createBlobsImage(frame,cv::Scalar(0,255,0));

		// print the 5 "most probable" blobs on image (blue)
		if (imagePoints.size() > 0) {
			for (int i = 0; i < NO_LEDS; i++) {
				cv::circle(frame,imagePoints[i], 5, cv::Scalar(255,0,0), 3);
			}
		}

		PnP.drawOverFrame(frame);

#endif /*DEBUG_VIDEO -or- SAVEOFF_FRAMES*/

#ifdef SAVEOFF_FRAMES
		frameCount++;
		if (frameCount % frameSkip == 0) {

			// add a frame number label to image
			std::stringstream frameNoStr;
			frameNoStr << "Frame # " << frameCount;
			cv::putText(frame,frameNoStr.str(),cv::Point2f(20,20),
					FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255));


			// save off the file
			frameNoStr.str(""); // clear the string
			frameNoStr << imageSavepath << "/frame_" << frameCount << ".jpg";
			cv::imwrite(frameNoStr.str(), frame);

		}
#endif /*SAVEOFF_FRAMES*/

#ifdef DEBUG_VIDEO
		cv::imshow("DEBUG_VIDEO",frame);
		cv::waitKey(1);
#endif /*DEBUG_VIDEO*/

#ifdef DEBUG_STDOUT
		// display the results of the blob finding
		std::cout << "Number of blobs found -- " << imagePoints.size() << '\n';
		//std::cout << "Image Points: " << imagePoints << std::endl;

		// Print state to standard output
		std::cout << "Estimated Pose:  " << "[";
		if (PnP.is_current) {
		std::copy(poseState.begin(), poseState.end()-1, std::ostream_iterator<double>(std::cout, ", "));
		std::cout << (poseState.back());
		std::cout <<  "]\n";
		std::cout << "   in " << poseIters << " iterations\n";
		std::cout << "   error: " << poseErr << "\n";
		} else{
			std::cout << "  --- Could not solve PnP ---  ";
			std::cout <<  "]\n";
		}


		// compute and display the framerate (moving average)
		std::cout << "Average FPS:  " << FPSclk.fps() << "\n";
		std::cout << std::endl;
#endif

	}
	while (true);  // TODO: Modify loop to look for keypress to terminate program

	/////////////////////////////////////////////////////////////////////////////////
	// END MAIN LOOP
	/////////////////////////////////////////////////////////////////////////////////

	std::cout << "Main loop completed successfully" << std::endl;


	return 0;

}



void writeout_blobfile_header(const char* blobFilename,const CustomBlobDetector::Params blobParams){

	std::cout << "   creating a logfile to hold blob finding data" << std::endl;

	// open file for printing to
	bFile = fopen(blobFilename,"w");

	fprintf(bFile,"%15s%15s%15s%15s%15s%15s%15s\n","weights:","","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(bFile,"%15s%15s%15d%15d%15d%15d%15s\n","(0-255)","",blobParams.w_Circularity,blobParams.w_InertiaRatio,
			blobParams.w_Convexity,blobParams.w_BlobColor,"--");

	fprintf(bFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","FilterBy:","error","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(bFile,"%15s%15d%15d%15d%15d%15d%15d\n","",blobParams.filterByError,blobParams.filterByCircularity,blobParams.filterByInertia,
			blobParams.filterByConvexity,blobParams.filterByColor,blobParams.filterByArea);

	fprintf(bFile,"\n%15s%15d%15.3f%15.3f%15.3f%15.3f%15.3f\n","min",0,blobParams.minCircularity,blobParams.minInertiaRatio,
			blobParams.minConvexity,blobParams.minThreshold,blobParams.minArea);
	fprintf(bFile,"%15s%15.3f%15.3f%15.3f%15.3f%15.3f%15.3f\n","max",blobParams.maxError,blobParams.maxCircularity,blobParams.maxInertiaRatio,
			blobParams.maxConvexity,255.0,blobParams.maxArea);

	fprintf(bFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","Targets:","","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(bFile,"%15s%15s%15.3f%15.3f%15.3f%15d%15s\n","","",blobParams.targetCircularity,blobParams.targetInertiaRatio,
			blobParams.targetConvexity,blobParams.targetBlobColor,"--");

	fprintf(bFile,"\n%15s%15s%15s%15s%15s%15s%15s%15s%15s\n","blobNo","TotalError","circularity","inertiaRatio",
			"convexity","blobColor","area","imageXpt","imageYpt");
	fclose(bFile);
}

void writeout_posefile_header(const char* poseFilename){

	std::cout << "   creating a logfile to pose estimate data" << std::endl;

	// open file for printing to
	pFile = fopen(poseFilename,"w");

	fprintf(pFile,"%-30s %s\n","state:","[dx, dy, dz, dphi, dtheta, dpsi]");
	fprintf(pFile,"%-30s %s\n","RpjErr:","");
	fprintf(pFile,"%-30s %s\n","Iter:","-- Nuber of iterations (swaps + correlations");
	fprintf(pFile,"%-30s %s\n\n","LEDimagePts:","[x1,y1,x2,y2,...,x5,y5] -- Correlated image points for LEDs");

	fprintf(pFile,"%-30s %s\n","Camera Calibration File:",camDataFilename.c_str());
	fprintf(pFile,"%-30s %s\n\n","3-D Model Points File:",modelPointsFilename);

	fprintf(pFile,"%-30s %-15.6f\n","Primary Error Tolerance:",POSE_ERR_TOL);
	fprintf(pFile,"%-30s %-15.6f\n\n\n","Secondary Error Tolerance:",SECONDARY_POSE_ERR_TOL);

	fprintf(pFile,"%-53s   %9s   %4s   %-79s\n",
			"state","RpjErr","Iter","ImagePts");
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
