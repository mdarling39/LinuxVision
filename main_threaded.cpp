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

struct v4l2Parms parms;
#define BUF_SZ 4
struct v4l2_buffer buf;
void* buff_ptr;
struct user_buffer_t{
    void* ptr[BUF_SZ];
    int buf_idx;
    int buf_last;
} user_buffer;


Mat frame(Size(IM_WIDTH,IM_HEIGHT),CV_8UC3);
Mat gray(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);
Mat binary(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);

vector<vector<Point> > contours;

FPSCounter fps(15);

void custom_v4l2_init(void*);

/// Multithreading variables
void* capture(void*);
void* processing(void*);
pthread_mutex_t framelock_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t done_saving_frame = PTHREAD_COND_INITIALIZER;
pthread_cond_t done_using_frame = PTHREAD_COND_INITIALIZER;

int main()
{
    // Initialize pthreads
    pthread_t capture_thread;
    pthread_t processing_thread;

    pthread_create(&capture_thread,NULL,capture,NULL);
    pthread_create(&processing_thread,NULL,processing,NULL);

    pthread_join(capture_thread,NULL); // Let the capture thread end the program

    cout << "DONE" << endl;

    return 0;
}

void *capture(void*)
{
    // Configure camera properties

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

    // Cycle through and fill up the image pointers
    for (int i=0; i<2*BUF_SZ; i++)
    {
        while(0 != v4l2_wait_for_data(&parms)){}
        v4l2_fill_buffer(&parms, &buf, &user_buffer.ptr[buf.index]);
        v4l2_queue_buffer(&parms, &buf);
    }

    //  Capture frames
    while(1)
    {
        if ( 0 != v4l2_wait_for_data(&parms))   // calls select()
            continue;                           // retry on EAGAIN

        pthread_mutex_lock(&framelock_mutex);
        user_buffer.buf_idx = buf.index;
        user_buffer.buf_last = (buf.index - 1) % BUF_SZ;
        v4l2_fill_buffer(&parms, &buf, &user_buffer.ptr[user_buffer.buf_idx]); // dequeue buffer
        v4l2_queue_buffer(&parms, &buf);
        pthread_cond_broadcast(&done_saving_frame);
        pthread_mutex_unlock(&framelock_mutex);

//        double fps_cnt=fps.fps();
//        cout << "\r" "FPS: " << fps_cnt << flush;

    }

    v4l2_stop_capturing(&parms);
    v4l2_uninit_device(&parms);
    v4l2_close_device(&parms);

    pthread_exit(NULL);
}

void *processing(void*)
{
sleep(3); //Ensure that the capture thread starts us off
while(1)
{

    /// TODO:  Include some kind of protection to make sure we dont' repeatedly process the same frame
    pthread_mutex_lock(&framelock_mutex);
    // Decode JPEG image stored in the most recently dequeued buffer
    v4l2_process_image(frame, user_buffer.ptr[user_buffer.buf_last]);
    pthread_mutex_unlock(&framelock_mutex);

    const int mixCh[]= {2,0};
    mixChannels(&frame,1,&gray,1,mixCh,1);  // For now, OpenCV's implementation is faster
    //gray = gray.clone(); // force a hard copy (This might be a little bit expensive)
                         // consider implementing a circular image buffer to skip this step


    threshold(gray,binary,215,255,THRESH_BINARY);

    /// Optional dilation
    cv::Mat kernel(5,5,CV_8UC1,Scalar(0));
    cv::circle(kernel,Point(2,2),2,Scalar(255));
    dilate(binary,binary,kernel);

    findContours(binary,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    double fps_cnt=fps.fps();
    printf("\r  FPS:  %2.2f   contours: %4d",fps_cnt,contours.size());
    fflush(stdout);

    //imshow("drawing",gray);
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
