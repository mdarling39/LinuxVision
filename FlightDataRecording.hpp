#ifndef FLIGHTDATARECORDING_HPP_INCLUDED
#define FLIGHTDATARECORDING_HPP_INCLUDED

#include "Global.hpp"
#include "Config.hpp"
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <sstream>

/// Used for saving sample frames
#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Helper Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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


void saveDebugFrame(cv::Mat &frame, char* path)
{
    static int frameCounter = 0;
    static struct timespec now, last;
    double elapsed;

    if (frameCounter == 0)
    {
        clock_gettime(CLOCK_MONOTONIC, &last);
        frameCounter++;
    }
    clock_gettime(CLOCK_MONOTONIC, &now);
    elapsed =  (now.tv_sec - last.tv_sec) * 1000.0;
    elapsed += (now.tv_nsec - last.tv_nsec) / 1000000.0;

    if (elapsed > frameSkip_ms)
    {
        std::stringstream filename;
        filename << path << "/frame_" << (int)frameCounter << ".jpg";
        imwrite(filename.str(), frame);

        frameCounter++;
        last = now;
    }
}

#endif // FLIGHTDATARECORDING_HPP_INCLUDED
