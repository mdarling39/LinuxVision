#include "CamObj.hpp"
#include "FPSCounter.hpp"
#include "ProfilerTool.h"
#include "morph.hpp"


#include <opencv2/opencv.hpp>
#include <vector>
#include <bitset>
#include <iostream>

using namespace std;
using namespace cv;

#ifdef __arm__
const int DEVICE=0;
#else
const int DEVICE=1;
#endif // __arm__
const int IM_WIDTH = 640;
const int IM_HEIGHT = 480;
const unsigned char THRESHOLD = 225;
const unsigned char THRESHOLD_MAXVAL = 255;
#define LIVE_CAPTURE 1

#if LIVE_CAPTURE
static Mat frame(IM_HEIGHT,IM_WIDTH,CV_8UC3);
#else
Mat frame = imread("/home/mdarling/Desktop/CompleteVision_MAIN/SampleImage.jpeg",CV_LOAD_IMAGE_COLOR);
#endif
static Mat gray(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);
static Mat binary(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

ProfilerTool timerA, timerB;
time_t time_tic,time_toc;
double time_tot = 0;
FPSCounter fps(15);


int main()
{

    // open the camera
    CamObj cap;
    cap.set_image_size(IM_WIDTH,IM_HEIGHT);
    cap.set_framerate(30);
    cap.open(DEVICE);

    vector<KeyPoint> keypoints;
    vector<Vec4i> hierarchy;
    vector<Vec3f> circles;

    int prof_i=0;
    for (int iter=0; iter<2500; iter++)
    {
        if (prof_i++ == 150)
        {
            printf ("\nCapture time: %f milliseconds,time.\n",(float)time_tot * 1000.0);
            printf ("TimerA took: %d clicks (%f milliseconds).\n",(int)timerA.n_clocks(),timerA.ms());
            printf ("TimerB took: %d clicks (%f milliseconds).\n",(int)timerB.n_clocks(),timerB.ms());
            return 0;
        }

        // capture an image
        time(&time_tic);
#if LIVE_CAPTURE
        cap >> frame;
#else
        resize(frame,frame,Size(IM_WIDTH,IM_HEIGHT));
#endif

        time(&time_toc);
        time_tot += difftime(time_toc,time_tic);

        double fps_cnt=fps.fps();
        if ((iter%15) == 0)
            cout << "\r" "FPS: " << fps_cnt << flush;

        timerA.tic();
        const int mixCh[]= {2,0};
        mixChannels(&frame,1,&gray,1,mixCh,1);  // For now, OpenCV's implementation is faster

        threshold(gray,binary,THRESHOLD,255,THRESH_BINARY);  // OpenCV is faster at thresholding too

        timerB.tic();
        findContours(binary,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        timerB.toc();
        timerA.toc();



        /// Draw the contours image
        Mat drawing = Mat::zeros(480,640,CV_8UC1);
        for (int i=0; i<contours.size(); i++)
        {
            drawContours(drawing,contours,i,Scalar(255));
        }
        imshow("window",drawing);
        waitKey(1);


        /*
        char name[50];
        sprintf(name,"frame%d.jpg",prof_i);
        imwrite(name,binary);
        cout << name << endl;
        */
        continue;

    return 0;
    }
}
