#include "LED_Detector.hpp"

using namespace std;
using namespace cv;

void LED_Detector::findLEDs(const cv::Mat& RGBImage, cv::Mat &grayImage, cv::Mat &binaryImage, vector<Point2f> &leds, const LED_Detector::Params &params)
{
    // Extract the red channel and save as gray image
    const int mixCh[]= {2,0};
    mixChannels(&RGBImage,1,&grayImage,1,mixCh,1);

    // threshold the gray image
    threshold(grayImage, binaryImage, params.threshold, 255, THRESH_BINARY);

    // dilate to close holes
    //dilate(binaryImage,binaryImage,Mat());

    // detect contours -- use CV_RETR_EXTERNAL to avoid returning holes
    static vector<vector<Point> > contours;
    findContours(binaryImage,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    /// We now have a vectorized contours object: vector<vector<Point> > contours

    static LED_Detector::contourStructObj contourStruct;
    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {

        Moments moms = moments(Mat(contours[contourIdx]));
        contourStruct.center = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (params.filterByArea)
        {
            float area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
            contourStruct.area = area;
        }

        if (params.filterByCircularity)
        {
            float area = moms.m00;
            float perimeter = arcLength(Mat(contours[contourIdx]),true);
            float ratio = 4 * CV_PI * area / (perimeter * perimeter);
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
            contourStruct.circularity = ratio;
        }

        if (params.filterByConvexity)
        {
            vector<Point> hull;
            convexHull(Mat(contours[contourIdx]), hull);
            float area = contourArea(Mat(contours[contourIdx]));
            float hullArea = contourArea(Mat(hull));
            float ratio = area / hullArea;
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
            contourStruct.convexity = ratio;
        }

        leds.push_back(contourStruct.center);
    }


    Mat debugImage = RGBImage.clone();
    for (int i=0; i<contours.size(); i++)
    {
        drawContours(debugImage,contours,i,Scalar(255,0,0),-1);
    }

    for (int i=0; i<leds.size(); i++)
    {
        circle(debugImage,leds[i],4,Scalar(30,30,225),3);
    }

    imshow("LED_Detector_Debug",debugImage);
    waitKey(1);

}
