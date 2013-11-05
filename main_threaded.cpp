
#include "Global.hpp"
#include "Config.hpp"
#include "v4l2_c.h"  // C-friendly version of CamObj.hpp so we can do multithreading
#include "FPSCounter.hpp"
#include "LED_Detector.hpp"
#include "PnPObj.hpp"
#include "BBBSerial.h"
#include "FlightDataRecording.hpp"
#include "ProfilerTool.h"  // ProfilerTool class to make profiling code easier


#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iterator>
#include <pthread.h>    // multithreading

#define PRINT_LINEBREAK()\
 printf("--------------------------------------------------------------------------------\n")

using namespace std;
using namespace cv;


/// ////////// Global Variables ////////// ///

// Camera parameters and buffers
struct v4l2Parms parms;
struct v4l2_buffer buf;
void* buff_ptr;
struct user_buffer_t{
    void* ptr[BUF_SZ];
    int buf_idx;
    int buf_last;
} user_buffer;

// OpenCV image objects
Mat frame(Size(IM_WIDTH,IM_HEIGHT),CV_8UC3);
Mat gray(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);
Mat binary(Size(IM_WIDTH,IM_HEIGHT),CV_8UC1);

vector<double> poseState(6);
vector <double> reportState(6,NAN); // contains the previous "good" state
double poseErr;

// User-defined class objects
LED_Detector::Params DetectorParams;
LED_Detector Detector;
PnPObj PnP;         // Correlates LEDs w/ model points and computes UAV localization estimate
FPSCounter fps(15); // Computes real-time frame rate
#if ARM
        BBBSerial Serial;
#endif
/// ////////////////////////////////////// ///



/// Multithreading variable declarations
void* capture(void*);
void* processing(void*);
pthread_mutex_t framelock_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t done_saving_frame = PTHREAD_COND_INITIALIZER;
pthread_cond_t done_using_frame = PTHREAD_COND_INITIALIZER;

void selfIdentifySystem(void);

int main()
{
    // Print the system to stdout
    selfIdentifySystem();

    // Initialize threshold object
    initializeFeatureDetectObj(DetectorParams);
    cout << "Feature detector initialized." << endl;

    // Read camera intrinsic properties
    PnP.setCamProps(camDataFilename);
    cout << "Camera properties read-in." << endl;

    // Read 3-D model geometry
    PnP.setModelPoints(modelPointsFilename);
    cout << "3-D model geometry read-in." << endl;

    // Check that serial ports were initialized
#if ARM
    // This is actually done above main() for global scope
    cout << "Serial ports for BeagleBone Black initialized." << endl;
#endif /* ARM */

    // Image directory is initialized after a frame is captured in the capture thread

#ifdef DEBUG_VIDEO
    // Much faster if we create a named window here
    cv::namedWindow("DEBUG_VIDEO");
    cout << "   real-time debugging window created" << endl;
#endif /*DEBUG_VIDEO*/

    // (Camera initialization will occur in the "capture" thread)



    // Initialize pthreads
    pthread_t capture_thread;
    pthread_t processing_thread;

    pthread_create(&capture_thread,NULL,capture,NULL);
    pthread_create(&processing_thread,NULL,processing,NULL);

    pthread_join(capture_thread,NULL); // Let the capture thread end the program

    cout << "Program ended" << endl;
    return 0;
}



void *capture(void*)
{

    // Configure camera properties
    v4l2_set_defalut_parms(&parms);
    char devName[15];
    sprintf(devName,"/dev/video%d",DEVICE);
    strcpy(parms.dev_name,devName);
    parms.width = IM_WIDTH;
    parms.height = IM_HEIGHT;
    parms.fps = IM_FPS;
    parms.timeout = 1;
    parms.customInitFcn = &custom_v4l2_init;

    // Open + initialize the camera for capturing
    v4l2_open_device(&parms);
    v4l2_init_device(&parms);
    v4l2_start_capturing(&parms);

    // Populate the circular buffer of pointers to image data
    for (int i=0; i<2*BUF_SZ; i++)
    {
        while(0 != v4l2_wait_for_data(&parms)){}
        v4l2_fill_buffer(&parms, &buf, &user_buffer.ptr[buf.index]);
        v4l2_queue_buffer(&parms, &buf);
    }

    // make final attempt at setting the camera's parameters
    custom_v4l2_init(&parms);

    cout << "Camera initialized and capturing." << endl;

        // Initialize and flight recording tools
#ifdef SAVEOFF_FRAMES
    // clear the folder and write a reference RGB frame
    v4l2_process_image(frame, user_buffer.ptr[user_buffer.buf_last]);
    setup_images_dir(imageSavepath,frame);
    cout << "   images directory created" << endl;
#endif /*SAVEOFF_FRAMES*/
    PRINT_LINEBREAK();



    //  Capture frames indefinitely
    while(1)
    {
        if ( 0 != v4l2_wait_for_data(&parms))   // calls select() -- waits for data to be ready
            continue;                           // retry on EAGAIN

        pthread_mutex_lock(&framelock_mutex);  // protect from other threads modifying image buffers
        user_buffer.buf_idx = buf.index;
        user_buffer.buf_last = (buf.index - 1) % BUF_SZ;
        v4l2_fill_buffer(&parms, &buf, &user_buffer.ptr[user_buffer.buf_idx]); // dequeue buffer
        v4l2_queue_buffer(&parms, &buf);
        pthread_cond_broadcast(&done_saving_frame);
        pthread_mutex_unlock(&framelock_mutex);
    }

    v4l2_stop_capturing(&parms);
    v4l2_uninit_device(&parms);
    v4l2_close_device(&parms);

    pthread_exit(NULL);
}



void *processing(void*)
{
sleep(3); // Ensure that the capture thread has time to initialize and fill buffer
PnP.is_current = false;
while(1)
{

    /// TODO:  Include some kind of protection to make sure we don't repeatedly process the same frame
    pthread_mutex_lock(&framelock_mutex);
    // Decode JPEG image stored in the most recently dequeued buffer
    v4l2_process_image(frame, user_buffer.ptr[user_buffer.buf_last]);
    pthread_mutex_unlock(&framelock_mutex);


    vector<Point2f> imagePoints;
    bool preCorrelated =
        Detector.findLEDs(frame,gray,binary,imagePoints,DetectorParams,PnP.is_current,PnP.projImagePoints);

    // Compute pose estimate
    int poseIters = PnP.localizeUAV(imagePoints, poseState, poseErr, 6, POSE_ERR_TOL, SECONDARY_POSE_ERR_TOL);
    if ( poseIters > 0 && checkSanity(poseState)>0 )
    {
            PnP.is_current = true;
            reportState = poseState;
    } else {
        PnP.is_current = false;
    }


    // send pose estimate to autopilot
#if ARM
    Serial.writeData(poseState);
#endif





/// ////////// DEBUGGING SPECIFIC OPTIONS ////////// ///

    /// Print fps and pose estimate to console in real-time
#ifdef POSE_TO_CONSOLE
    double fps_cnt=fps.fps();
    printf("\e[J  FPS:  %6.2f        # of detected features: %4d\n",fps_cnt,imagePoints.size());
    printf("  Pose Estimate:%7s  %7s  %7s  %7s  %7s  %7s   [in/deg]\n","x","y","z","roll","pitch","yaw");
    printf("                %7.1f  %7.1f  %7.1f  %7.1f  %7.1f  %7.1f",
    reportState[0], reportState[1], reportState[2],
    reportState[3], reportState[4], reportState[5]);
    if (!PnP.is_current)
        printf("   ZOH");
    printf("\r\e[2A"); // move cursor
    fflush(stdout);
#endif /* POSE_TO_CONSOLE */

#ifdef DEBUG_VIDEO
        // print blobs on image (green)
		//thresh.createBlobsImage(frame,cv::Scalar(0,255,0));

		// print the 5 "most probable" blobs on image (blue)
		if (imagePoints.size() > 0) {
			for (int i = 0; i < imagePoints.size(); i++) {
				cv::circle(frame,imagePoints[i], 5, cv::Scalar(255,0,0), 3);
			}
		}

		PnP.drawOverFrame(frame);
		imshow("DEBUG_VIDEO",frame);
		waitKey(1);
#endif /* DEBUG_VIDEO */

#ifdef SAVEOFF_FRAMES
    saveDebugFrame(frame, imageSavepath);
#endif /* SAVEOFF_FRAMES */

/// ////////// DEBUGGING SPECIFIC OPTIONS ////////// ///

}
pthread_exit(NULL);
}




void selfIdentifySystem(void)
{
std::cout << "System is:   ";
#if OSX
    std::cerr << "OSX -- no longer supported" << std::endl;
#elif LINUX
    std::cout << "LINUX" << std::endl;
#else
    std::cerr << "Unknown System!" << std::endl;
#endif
#if ARM
    std::cout << "ARM" << std::endl;
#endif
}
