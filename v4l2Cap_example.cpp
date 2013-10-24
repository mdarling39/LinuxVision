#include <iostream>
#include <opencv2/opencv.hpp>
#include "v4l2Cap.h"
#include "OCVCapture.h"

int example_main()
{
    v4l2Cap cap;
    cv::Mat image;

    cap.set_dev_name("/dev/video1");        // set the camera
    cap.set_pix_fmt(V4L2_PIX_FMT_MJPEG);    // chose format

// CAN WE CAPTURE FRAMES THIS LARGE ON BBB?
    cap.set_width(800);                     // set frame dims.
    cap.set_height(600);

    cap.set_fps(30);                        // set framerate
    cap.customInit = true;                  // use custom initialization

    cap.open_device();

    cv::namedWindow("Window");
    for (int i=0; i<10000; i++)
    {
        cap.grab_frame(image);
        imshow("Window",image);
        cv::waitKey(1);
        std::cout << i << std::endl;
    }

    cap.close_device();
    return 0;
}
