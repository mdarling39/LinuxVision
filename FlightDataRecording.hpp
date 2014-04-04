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
    char filename[500];
    strcpy(filename,imageDirname);
    strcat(filename,"/frame_0.jpg");
    imwrite(filename, frame);
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
        char filename[500];
        sprintf(filename,"%s/frame_%04d.jpg",path,(int)frameCounter);
        imwrite(filename, frame);

        frameCounter++;
        last = now;
    }
}

void recordLogData(FILE* fd, std::vector<double> &filtered, std::vector<double> &unfiltered, double reprojErr, bool &status)
{
    static struct timespec now, last;
    double time;
    clock_gettime(CLOCK_MONOTONIC, &now);

    static bool first_exec = true;
    if (first_exec) {
        last = now;
        first_exec = false;
    }

    time =  (now.tv_sec - last.tv_sec) * 1000.0;
    time += (now.tv_nsec - last.tv_nsec) / 1000000.0;
    time /= 1000; // convert ms to seconds

    // Print time
    fprintf(fd, "%.4f,",time);

    // Print filtered state
    fprintf(fd, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",filtered[0],filtered[1],filtered[2],
        filtered[3],filtered[4],filtered[5]);

    // Print unfiltered state
    fprintf(fd, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",unfiltered[0],unfiltered[1],unfiltered[2],
        unfiltered[3],unfiltered[4],unfiltered[5]);

    // Print the reprojection error
    fprintf(fd, "%.6f,",reprojErr);

    // Print the status
    fprintf(fd, "%d\n", status);
}

#endif // FLIGHTDATARECORDING_HPP_INCLUDED
