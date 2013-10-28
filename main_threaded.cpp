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
pthread_mutex_t taketurns_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t done_saving_frame = PTHREAD_COND_INITIALIZER;
pthread_cond_t done_using_frame = PTHREAD_COND_INITIALIZER;
pthread_cond_t done_capturing = PTHREAD_COND_INITIALIZER;
pthread_cond_t done_processing = PTHREAD_COND_INITIALIZER;

int BUFF_FULL = 0;
int DONE_PROCESSING = 1;
int DONE_CAPTURING = 0;

int main()
{
    // Initialize pthreads
    pthread_t capture_thread;
    pthread_t processing_thread;

    pthread_create(&capture_thread,NULL,capture,NULL);
    pthread_create(&processing_thread,NULL,processing,NULL);

    pthread_join(capture_thread,NULL); // Let the capture thread end the program

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

        time(&time_tic);
        if ( 0 != v4l2_wait_for_data(&parms))   // calls select()
            continue;                           // retry on EAGAIN

        struct v4l2_buffer buf;
        void* buff_ptr;

        v4l2_fill_buffer(&parms, &buf, &buff_ptr); // dequeue buffer

        ///Show the rate at which we are taking images from the camera
        //double fps_cnt=fps.fps();
        //cout << "\r" "FPS: " << fps_cnt << flush;

        // May not want to wait for processing function to finish.
        if ( DONE_PROCESSING < 1)
        {
            v4l2_queue_buffer(&parms, &buf);
            --iters; // don't count this iteration
            continue;
        }

        // Wait until processing thread is done accessing the buffer
        pthread_mutex_lock(&framelock_mutex);
        if (BUFF_FULL > 0)
            pthread_cond_wait(&done_using_frame,&framelock_mutex);

        // Decode the JPEG image and store to the framebuffer
        v4l2_process_image(frame, buff_ptr);
        time(&time_toc);
        time_tot += difftime(time_toc, time_tic);

        BUFF_FULL = 1;
        pthread_cond_broadcast(&done_saving_frame);
        pthread_mutex_unlock(&framelock_mutex);

        v4l2_queue_buffer(&parms, &buf); // queue the buffer for next capture

        pthread_mutex_lock(&taketurns_mutex);
        DONE_CAPTURING = 1; DONE_PROCESSING = 0;
        pthread_cond_broadcast(&done_capturing);
        pthread_mutex_unlock(&taketurns_mutex);
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
    // Wait until there is a new image worth processing
    pthread_mutex_lock(&taketurns_mutex);
    if (DONE_CAPTURING < 1)
        pthread_cond_wait(&done_capturing,&taketurns_mutex);
    pthread_mutex_unlock(&taketurns_mutex);


    pthread_mutex_lock(&framelock_mutex);
    if (BUFF_FULL < 1)
        pthread_cond_wait(&done_saving_frame,&framelock_mutex);

    timerA.tic();
    const int mixCh[]= {2,0};
    mixChannels(&frame,1,&gray,1,mixCh,1);  // For now, OpenCV's implementation is faster
    //gray = gray.clone(); // force a hard copy (This might be a little bit expensive)
                         // consider implementing a circular image buffer to skip this step



    BUFF_FULL = 0;
    pthread_cond_broadcast(&done_using_frame);
    pthread_mutex_unlock(&framelock_mutex);

    timerB.tic();
    threshold(gray,binary,215,255,THRESH_BINARY);
    const cv::Mat kernel(3,3,CV_8UC1,Scalar(255));
    findContours(binary,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    timerB.toc();
    timerA.toc();

    double fps_cnt=fps.fps();
    cout << "\r" "FPS: " << fps_cnt << flush;

    //imshow("drawing",gray);
    //waitKey(1);


    pthread_mutex_lock(&taketurns_mutex);
    DONE_PROCESSING = 1; DONE_CAPTURING = 0;
    pthread_cond_broadcast(&done_processing);
    pthread_mutex_unlock(&taketurns_mutex);
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
