#include "LED_Detector.hpp"

using namespace std;
using namespace cv;

/// Passes default arguments for args 6 and 7
void LED_Detector::findLEDs(const cv::Mat& RGBImage, cv::Mat &grayImage, cv::Mat &binaryImage,
    vector<Point2f> &leds, const LED_Detector::Params &params)
{
    LED_Detector::findLEDs(RGBImage, grayImage, binaryImage,
        leds, params, false, vector<Point2f>(NO_LEDS));
}

bool LED_Detector::findLEDs(const cv::Mat& RGBImage, cv::Mat &grayImage, cv::Mat &binaryImage,
    vector<Point2f> &leds, const LED_Detector::Params &params,
    bool havePreviousState, const vector<Point2f> &reprojImgPts)
{
    // Extract the red channel and save as gray image
    const int mixCh[]= {2,0};
    mixChannels(&RGBImage,1,&grayImage,1,mixCh,1);

    // threshold the gray image
    threshold(grayImage, binaryImage, params.threshold, 255, THRESH_BINARY);

    // detect contours -- use CV_RETR_EXTERNAL to avoid returning holes
    vector<vector<Point> > contours;
    findContours(binaryImage,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);




    /// We now have a vectorized contours object: vector<vector<Point> > contours
    /// Loop through each contour and filter/classify it by its properties

    static LED_Detector::contourStructObj contourStruct;
    vector <LED_Detector::contourStructObj> contourStructVec;
    int n_detected = 0;

    if (havePreviousState) // Need to initialize proximities
    {
        proximityVec.resize(NO_LEDS);

        // vector to keep closest points in
        for (int i=0; i<NO_LEDS; i++)
        {
            proximityVec[i].center = Point2f(NAN,NAN);
            proximityVec[i].radius = INFINITY;   // Record that we dont' see any nearby points
        }
        float span_sq = pow((reprojImgPts[NO_LEDS-1].x - reprojImgPts[0].x),2) //compute the span to normalize by later
            + pow((reprojImgPts[NO_LEDS-1].y - reprojImgPts[0].y),2);
        span = sqrt(span_sq);
    }

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        /// Reduce resolution
        //approxPolyDP(contours[contourIdx], contours[contourIdx], 2, true);

        /// Use minEnclosingCircle for blob area
        minEnclosingCircle(contours[contourIdx], contourStruct.center, contourStruct.radius);
        contourStruct.area = CV_PI * (contourStruct.radius * contourStruct.radius);

        if (params.filterByArea)
        {
            if (contourStruct.area < params.minArea || contourStruct.area >= params.maxArea)
                continue;
        }

        if (params.filterByCircularity)
        {
            Moments moms = moments(Mat(contours[contourIdx]));
            static float area = moms.m00;
            static float ratio = area / contourStruct.area;  // (blob area) / (min enclosing circle area)
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
            contourStruct.circularity = ratio;
        }

        // minimum bounding ellipse
        if (params.filterByAspectRatio)
        {
            if (contours[contourIdx].size() >=5 ) // need at least 5 elements
            {
                static RotatedRect rRect = fitEllipse(contours[contourIdx]);
                static float width = rRect.size.width;
                static float height = rRect.size.height;
                static float AR = (width > height) ? (height/width) : (width/height);
                if (AR < params.minAspectRatio || AR >= params.maxAspectRatio)
                    continue;
                contourStruct.aspectRatio = AR;
                contourStruct.center = rRect.center;

                // draw the detected ellipse
                ellipse(grayImage, rRect, Scalar(0,255,0), 2, 8);
            }
        }

        // compute the average RGB color of the blob
        if (params.sortByColor || params.filterByColor)
        {
            static unsigned int R,G,B;
            static Mat RGB(1,1,CV_32FC3);
            static Mat HSV(1,1,CV_32FC3);
            static unsigned int n=0;
            R = G = B = 0;

            // get the bounding rectangle
            Rect roi = boundingRect(contours[contourIdx]);

            // iterate over bounding rectangle
            for (int y=roi.y; y<roi.y + roi.height; y++){
                for(int x=roi.x; x< roi.x + roi.width; x++){
                    // Check if point lies inside contour
                    if (pointPolygonTest(contours[contourIdx], Point2f(x,y),false) > 0){
                        B += RGBImage.at<Vec3b>(y,x)[0];
                        G += RGBImage.at<Vec3b>(y,x)[1];
                        R += RGBImage.at<Vec3b>(y,x)[2];
                        n++;
                    }
                }
            }
            if (n < 1)  // We should have at least one pixel -- if not, give up
                continue;

            // Average colors inside conotur
            RGB.at<Vec3f>(0,0)[0] = B/n;
            RGB.at<Vec3f>(0,0)[1] = G/n;
            RGB.at<Vec3f>(0,0)[2] = R/n;
            cvtColor(RGB,HSV,CV_BGR2HSV);
            contourStruct.color = HSV.at<Vec3f>(0,0)[0]; // get the average Hue

            // compute difference from target
            contourStruct.color_delta = contourStruct.color - params.targetColor;
            if (contourStruct.color_delta > 180)
                contourStruct.color_delta -= 360;
            else if (contourStruct.color_delta < -180)
                contourStruct.color_delta += 360;
            contourStruct.color_delta = abs(contourStruct.color_delta);


            if (params.filterByColor)
            {
                if ( contourStruct.color_delta > params.maxColor )
                    continue;
            }


            if (havePreviousState)
            {
                // check each blob if in local raidus of previous image point
                for (int i=0; i<NO_LEDS; i++)
                {
                    float radius_sq = pow((contourStruct.center.x - reprojImgPts[i].x),2)
                        + pow((contourStruct.center.y - reprojImgPts[i].y),2);
                    float radius = sqrt(radius_sq);

                    // check if within local radius
                    if (radius < (params.localRadius * span / 100))
                    {
                        n_detected++;
                        if (radius < (proximityVec[i].radius)) // replace if better than current value
                        {
                            proximityVec[i].center = contourStruct.center;
                            proximityVec[i].radius = radius;
                        }
                    }
                }
            }

        }

        contourStructVec.push_back(contourStruct);
    }

    // If we don't detect at least 2 of the LEDs in current frame, don't use previous information
    // (prevents us from holding onto a past state that may not be true)
    if (n_detected < 3)
        havePreviousState = false;

    if (havePreviousState)
    {
        for (int i=0; i<NO_LEDS; i++)
        {
            // if we have an INF radius in this vector, replace it with the last reprojected image point
            if (proximityVec[i].radius == INFINITY)
            {
                proximityVec[i].center = reprojImgPts[i];
            }
            leds.push_back(proximityVec[i].center);
        }
    }
    else if (params.sortByColor)
    {
        sort(contourStructVec.begin(), contourStructVec.end(), LED_Detector::sortByColor);

        // populate the vector of image points from the contourStructVec
        int stopHere = (contourStructVec.size() > params.maxBlobs) ? params.maxBlobs : contourStructVec.size();
        for (int i=0; i<stopHere; i++)
        {
            leds.push_back(contourStructVec[i].center);
        }
    }


    #ifdef LED_DETECTOR_DEBUG
    Mat debugImage = RGBImage.clone();
    for (int i=0; i<contours.size(); i++)
    {
        drawContours(debugImage,contours,i,Scalar(153,255,255),-1);
        drawContours(debugImage,contours,i,Scalar(0,0,0),1);
    }

    for (int i=0; i<leds.size(); i++)
    {
        cout << leds[i] << endl;
        circle(debugImage,leds[i],3,Scalar(255,0,0),2);
        circle(debugImage,leds[i],6,Scalar(0,0,255),2);
        char msg[50];
        sprintf(msg,"%d",i);
        putText(debugImage,msg,leds[i],FONT_HERSHEY_PLAIN,2,Scalar(255,255,255),2);
    }


    imshow("LED_Detector_Debug",debugImage);
    waitKey(1);

    imshow("grayImage",grayImage);
    waitKey(0);
    #endif /* LED_DETECTOR_DEBUG */

    if (havePreviousState)
        return 1;
    else
        return -1;
}



