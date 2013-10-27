//#include "CamObj.hpp"
#include "v4l2_c.h"

#include "FPSCounter.hpp"
#include "ProfilerTool.h"
#include "morph.hpp"


#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <pthread.h>    // multithreading


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
Mat frame(Size(IM_WIDTH,IM_HEIGHT),CV_8UC3);
#else
Mat frame = imread("/home/mdarling/Desktop/CompleteVision_MAIN/SampleImage.jpeg",CV_LOAD_IMAGE_COLOR);
#endif
Mat gray(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);
Mat binary(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);


vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

ProfilerTool timerA, timerB;
time_t time_tic,time_toc;
double time_tot = 0;
FPSCounter fps(15);

void custom_v4l2_init(void*);

/// Multithreading variables
void* capture(void*);
void* processing(void*);
pthread_mutex_t framelock_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t done_saving_frame = PTHREAD_COND_INITIALIZER;
pthread_cond_t done_using_frame = PTHREAD_COND_INITIALIZER;

// enum type to protect access to the the "frame" cv::Mat object
enum access_t {
    READING,
    WRITING,
    FREE
};
access_t frameAccess = READING;

int main()
{

// Configure camera properties
    struct v4l2Parms parms;
    v4l2_set_defalut_parms(&parms);
    char devName[15];
    sprintf(devName,"/dev/video%d",DEVICE);
    strcpy(parms.dev_name,devName);
    parms.width = 640;
    parms.height = 480;
    parms.fps = 30;
    parms.timeout = 1;
    parms.customInitFcn = &custom_v4l2_init;

    // Open + initialize the camera for capturing
    v4l2_open_device(&parms);
    v4l2_init_device(&parms);
    v4l2_start_capturing(&parms);


    //  Capture frames
    for (int iters=0; iters<500; iters++)
    {
        /// Capture

        time(&time_tic);
        int rval = v4l2_wait_for_data(&parms);  // calls select()
        time(&time_toc);
        time_tot += difftime(time_toc, time_tic);

        if (0 != rval)
            continue;

        double fps_cnt=fps.fps();
        cout << "\r" "FPS: " << fps_cnt << flush;

        struct v4l2_buffer buf;
        void* buff_ptr;

        // The following three v4l2_ commands replace v4l2_grab_frame(&parms, frame);
        v4l2_fill_buffer(&parms, &buf, &buff_ptr); // dequeue buffer
        v4l2_process_image(frame, buff_ptr); /// THIS FUNCTION WRITES TO FRAME AND MUST BE PROTECTED!!!
        v4l2_queue_buffer(&parms, &buf);

        /// Process
        timerA.tic();
        const int mixCh[]= {2,0};
        mixChannels(&frame,1,&gray,1,mixCh,1);  // For now, OpenCV's implementation is faster
        //gray = gray.clone(); // force a hard copy (This might be a little bit expensive)
        // consider implementing a circular image buffer to skip this step

        timerB.tic();
        threshold(gray,binary,215,255,THRESH_BINARY);
        const cv::Mat kernel(3,3,CV_8UC1,Scalar(255));
        dilate(binary,binary,kernel);
        timerB.toc();
        timerA.toc();
    }

    v4l2_stop_capturing(&parms);
    v4l2_uninit_device(&parms);
    v4l2_close_device(&parms);


    // report timings
    printf ("\nCapture time: %f milliseconds,time.\n",(float)time_tot * 1000.0);
    printf ("TimerA took: %d clicks (%f milliseconds).\n",(int)timerA.n_clocks(),timerA.ms());
    printf ("TimerB took: %d clicks (%f milliseconds).\n",(int)timerB.n_clocks(),timerB.ms());

    return 0;
}

void *capture(void*)
{
    // Configure camera properties
    struct v4l2Parms parms;
    v4l2_set_defalut_parms(&parms);
    char devName[15];
    sprintf(devName,"/dev/video%d",DEVICE);
    strcpy(parms.dev_name,devName);
    parms.width = 640;
    parms.height = 480;
    parms.fps = 30;
    parms.timeout = 1;
    parms.customInitFcn = &custom_v4l2_init;

    // Open + initialize the camera for capturing
    v4l2_open_device(&parms);
    v4l2_init_device(&parms);
    v4l2_start_capturing(&parms);


    //  Capture frames
    for (int iters=0; iters<500; iters++)
    {

        if ( 0 != v4l2_wait_for_data(&parms))   // calls select()
            continue;

        struct v4l2_buffer buf;
        void* buff_ptr;

        // The following three v4l2_ commands replace v4l2_grab_frame(&parms, frame);
        v4l2_fill_buffer(&parms, &buf, &buff_ptr); // dequeue buffer
        pthread_mutex_lock(&framelock_mutex);
        if (frameAccess == READING)
            pthread_cond_wait(&done_using_frame,&framelock_mutex);

        time(&time_tic);
        v4l2_process_image(frame, buff_ptr); /// THIS FUNCTION WRITES TO FRAME AND MUST BE PROTECTED!!!
        time(&time_toc);
        time_tot += difftime(time_toc, time_tic);

        frameAccess = FREE;
        pthread_cond_broadcast(&done_saving_frame);
        pthread_mutex_unlock(&framelock_mutex);

        v4l2_queue_buffer(&parms, &buf);
    }

    v4l2_stop_capturing(&parms);
    v4l2_uninit_device(&parms);
    v4l2_close_device(&parms);

    pthread_exit(NULL);
}

void *processing(void*)
{
while(1)
{
    /// After this, we are working with the gray image
    pthread_mutex_lock(&framelock_mutex);
    if (frameAccess == WRITING)
        pthread_cond_wait(&done_saving_frame,&framelock_mutex);

    timerA.tic();
    const int mixCh[]= {2,0};
    mixChannels(&frame,1,&gray,1,mixCh,1);  // For now, OpenCV's implementation is faster
    //gray = gray.clone(); // force a hard copy (This might be a little bit expensive)
                         // consider implementing a circular image buffer to skip this step
    frameAccess = FREE;
    pthread_cond_broadcast(&done_using_frame);
    pthread_mutex_unlock(&framelock_mutex);

    timerB.tic();
    threshold(gray,binary,215,255,THRESH_BINARY);
    const cv::Mat kernel(3,3,CV_8UC1,Scalar(255));
    dilate(binary,binary,kernel);
    timerB.toc();
    timerA.toc();

    double fps_cnt=fps.fps();
    cout << "\r" "FPS: " << fps_cnt << flush;


    //imshow("drawing",binary);
    //waitKey(1);

}
pthread_exit(NULL);
}

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
    //set_parm(parm->fd, V4L2_CID_SATURATION,128);      // Adjust Saturation (0-255)
    set_parm(parm->fd, V4L2_CID_SHARPNESS,0);         // Blur the image to get smoother contours (0-255)

}
