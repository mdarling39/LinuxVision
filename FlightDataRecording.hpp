#ifndef FLIGHTDATARECORDING_HPP_INCLUDED
#define FLIGHTDATARECORDING_HPP_INCLUDED

/// Used for blobfile
#include<stdio.h>
FILE *bFile;
bool modeOn = false;

/// Used for posefile
FILE *pFile;

/// Used for saving sample frames
#include <sys/types.h>
#include <sys/stat.h>
unsigned int frameCount = 0;
unsigned int frameSkip = 30;




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Helper Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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

void initializeFlightDataRecorder(void)
{
#ifdef FOUNDBLOBS_TO_FILE
    // begin blobFile and writeout header
    writeout_blobfile_header(blobFilename,blobParams);
    cout << "   blobfile header written" << endl;
#endif /*FOUNDBLOBS_TO_FILE*/

#ifdef POSE_TO_FILE
    // begin poseFile and writeout header
    writeout_posefile_header(poseFilename);
    cout << "   posefile header written" << endl;
#endif /*POSE_TO_FILE*/

#ifdef SAVEOFF_FRAMES
    // clear the folder and write a reference RGB frame
    setup_images_dir(imageSavepath,frame);
    cout << "   images directory created" << endl;
#endif /*SAVEOFF_FRAMES*/

#ifdef DEBUG_VIDEO
    // Much faster if we create a named window here
    cv::namedWindow("DEBUG_VIDEO");
    cout << "   real-time debugging window created" << endl;
#endif /*DEBUG_VIDEO*/
}

#endif // FLIGHTDATARECORDING_HPP_INCLUDED
