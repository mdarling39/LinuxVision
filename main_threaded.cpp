
#include "Global.hpp"
#include "Config.hpp"
#include "v4l2_c.h"  // C-friendly version of CamObj.hpp so we can do multithreading
#include "FPSCounter.hpp"
#include "LED_Detector.hpp"
#include "PnPObj.hpp"
#include "BBBSerial.h"
#include "FlightDataRecording.hpp"
#include "ProfilerTool.h"  // ProfilerTool class to make profiling code easier
#include "ThresholdedKF.hpp"    // class to filter vehicle state


#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iterator>
#include <pthread.h>    // multithreading
#include <fstream>  // DEBUG: For saving data to file
#include <ctime>    // DEBUG: For recording time
#include <unistd.h> // DEBUG: Short delay
#include <signal.h> // DEBUG: Handle SIGINT signal (close files to avoid corrupted data)

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
ThresholdedKF::param_t KF_parms; // Kalman filter parameters
ThresholdedKF KF;   // Thresholded Kalman filter to reject outliers
FILE* logfd;        // file pointer to log file

// POSIX compliant threads
pthread_t capture_thread;
pthread_t processing_thread;

#if (!ARM)
struct timespec simulated_fps_tic, simulated_fps_toc; // clock objects to measure time
double simulated_fps_elapsed;
#endif //ARM


#ifdef SAVE_KF_DATA
// DEBUG: Create file to store Kalman filter debug data to as well as a timer
ofstream DEBUGFILE;
struct timespec DEBUG_tic, DEBUG_toc;
double DEBUG_elapsed = 0;
#endif // SAVE_KF_DATA


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

void signal_callback_handler(int signum)
{
    printf("\n\n\n%s","Interrupt signal received, closing files and shutting down.\n\n");
    fflush(stdout);


#ifdef SAVE_KF_DATA
    // Close the DEBUG file to avoid corrupted data
    DEBUGFILE.close();
#endif //SAVE_KF_DATA

#ifdef LOG_VISION_DATA
    if (logfd != NULL) fclose(logfd);
    fcloseall();
#endif


    // kill all pthreads
    pthread_cancel(processing_thread);
    pthread_cancel(capture_thread);
    //pthread_kill(processing_thread, SIGKILL);
    //pthread_kill(capture_thread, SIGKILL);

    printf("PROGRAM TERMINATED\n");
    fflush(stdout);

    exit(0);
}

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

    // Configure thresholded Kalman filter
    configThresholdedKF(KF_parms);
    KF.set_params(KF_parms);
    KF.forced_reset();
    cout << "Thresholded Kalman filter configured." << endl;

#ifdef SAVE_KF_DATA
    // DEBUG: Open file for writing data to and start clock
    DEBUGFILE.open("KF_Debug.txt");
    clock_gettime(CLOCK_MONOTONIC, &DEBUG_tic);
    clock_gettime(CLOCK_MONOTONIC, &simulated_fps_tic); // start timer for simulated FPS
#endif //SAVE_KF_DATA

    // Register signal and signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signal_callback_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);


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
    pthread_create(&capture_thread,NULL,capture,NULL);
    pthread_create(&processing_thread,NULL,processing,NULL);

    pthread_join(capture_thread, NULL); // Let the capture thread end the program
    pthread_join(processing_thread, NULL);

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

#ifdef LOG_VISION_DATA
    char logfilepath[500];
    strcpy(logfilepath,imageSavepath);
    strcat(logfilepath,"/visionLog.txt");
    logfd = fopen(logfilepath,"w");
#endif //LOG_VISION_DATA
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
    /// (if processing somehow goes faster than capturing -- not a problem on BBB right now)
    pthread_mutex_lock(&framelock_mutex);
    // Decode JPEG image stored in the most recently dequeued buffer
    v4l2_process_image(frame, user_buffer.ptr[user_buffer.buf_last]);
    pthread_mutex_unlock(&framelock_mutex);

#if (!ARM)
    // DEBUG: Wait until the desired amount of time has passed
    do
    {
        clock_gettime(CLOCK_MONOTONIC, &simulated_fps_toc);
        simulated_fps_elapsed = (simulated_fps_toc.tv_sec - simulated_fps_tic.tv_sec);
        simulated_fps_elapsed+= (simulated_fps_toc.tv_nsec - simulated_fps_tic.tv_nsec) / 1000000000.0;
    } while (simulated_fps_elapsed < 1.0/(simulated_fps));
    clock_gettime(CLOCK_MONOTONIC, &simulated_fps_tic);
#endif //ARM


    vector<Point2f> imagePoints;
    bool preCorrelated =
        Detector.findLEDs(frame,gray,binary,imagePoints,DetectorParams,PnP.is_current,PnP.projImagePoints);

    // Compute pose estimate
    int poseIters = PnP.localizeUAV(imagePoints, poseState, poseErr, 9, POSE_ERR_TOL, SECONDARY_POSE_ERR_TOL, preCorrelated);
    if ( poseIters > 0 && checkSanity(poseState) > 0 )
    {
            reportState = poseState;
            PnP.is_current = true;

    } else {
        PnP.is_current = false;
    }

#ifdef SAVE_KF_DATA
    // DEBUG: write Kalman filter inputs to file (including time)
    // compute time
    clock_gettime(CLOCK_MONOTONIC, &DEBUG_toc);
    DEBUG_elapsed = DEBUG_toc.tv_sec - DEBUG_tic.tv_sec;
    DEBUG_elapsed+= (DEBUG_toc.tv_nsec - DEBUG_tic.tv_nsec) / 1000000000.0;
    DEBUGFILE << DEBUG_elapsed << ",";
    // save input state
    for (int i=0; i<reportState.size(); i++)
        DEBUGFILE << reportState[i] << ",";
#endif //SAVE_KF_DATA


#ifdef EMPLOY_KF
    // Employ Kalman filter
    KF.predict(reportState.data());
    KF.correct();  // KF.correct() returns TRUE if not an outlier, FALSE if an outlier
    KF.get_state(reportState.data()); // Return the ESTIMATED state
#endif //EMPLOY_KF

#ifdef LOG_VISION_DATA
    recordLogData(logfd,reportState,poseState,poseErr,PnP.is_current);
#endif //LOG_VISION_DATA

#ifdef SAVE_KF_DATA
    // DEBUG: save output from Kalman filter
    for (int i=0; i<reportState.size()-1; i++)
        DEBUGFILE << reportState[i] << ",";
    DEBUGFILE << reportState.back() << "\n"; // don't write comma, proceed to newline
#endif //SAVE_KF_DATA

    // send pose estimate to autopilot
#if ARM
        Serial.writeData(reportState);
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

/// Temporarily moved here to save plain frames (without debug info)
//#ifdef SAVEOFF_FRAMES
//    saveDebugFrame(frame, imageSavepath);
//#endif /* SAVEOFF_FRAMES */

#ifdef DEBUG_VIDEO
		PnP.drawOverFrame(frame, reportState);
		imshow("DEBUG_VIDEO",frame);
		waitKey(1);
#endif /* DEBUG_VIDEO */

#if defined(SAVEOFF_FRAMES) && !defined(DEBUG_VIDEO) // don't repeat this step
    PnP.drawOverFrame(frame);
#endif
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
#elif ARM
    std::cout << "ARM" << std::endl;
#else
    std::cerr << "Unknown System!" << std::endl;
#endif
}
