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

#ifdef SHOW_BINARY_IMG
    imshow("binary",binaryImage);
#endif

    // detect contours -- use CV_RETR_EXTERNAL to avoid returning holes
    vector<vector<Point> > contours;
    findContours(binaryImage,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    /// Jump back to here if we can't use previous state
    not_detected_stop:


    /// We now have a vectorized contours object: vector<vector<Point> > contours
    /// Loop through each contour and filter/classify it by its properties

    static LED_Detector::contourStructObj contourStruct;
    vector <LED_Detector::contourStructObj> contourStructVec;
    int n_detected = 0;
    vector<Point2f> leaderROI_contour;       // vectorize the rectangle as a contour

    if (havePreviousState) // Need to initialize proximities
    {
        proximityVec.resize(NO_LEDS);  // This will resize 1st dimension to proximityVec[5][]
        for (int i=0; i<NO_LEDS; i++)
            proximityVec[i].resize(0);  // this will clear the contents of each subvector

        // Compute the wingspan based on reprojected image points from last frame
        float span_sq = pow((reprojImgPts[NO_LEDS-1].x - reprojImgPts[0].x),2)
            + pow((reprojImgPts[NO_LEDS-1].y - reprojImgPts[0].y),2);
        span = sqrt(span_sq);

        // Determine the bounding rectangle of the lead UAV and scale up a little bit
        RotatedRect leaderROI = minAreaRect(reprojImgPts);
        leaderROI.size.height += 90;
        leaderROI.size.width  += 90;
        Point2f verticies[4];

        // vectorize the bounding ROI
        leaderROI.points(verticies);
        leaderROI_contour.assign(verticies,verticies + 4);

        /// Uncomment to draw ROI on image frame
#if defined (DEBUG_VIDEO) || defined (SAVEOFF_FRAMES)
        for (int i_test=0; i_test<4; i_test++)
            line(frame, leaderROI_contour[i_test], leaderROI_contour[(i_test+1)%4], Scalar(255,255,255));
#endif
    }

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {

        /// Reduce resolution
        //approxPolyDP(contours[contourIdx], contours[contourIdx], 2, true);

        /// Use minEnclosingCircle for blob area
        minEnclosingCircle(contours[contourIdx], contourStruct.center, contourStruct.radius);
        contourStruct.area = CV_PI * (contourStruct.radius * contourStruct.radius);

        /// Uncomment to view all detected blobs
#if (!ARM)
        cv::circle(frame,contourStruct.center, 6, cv::Scalar(255,255,255), 3);
#endif
        /// Consider only the points that fall within a ROI around the last known position
        if (havePreviousState)
        {
            // if point does not lie within or lie on leaderROI RotatedRect, continue
            if (pointPolygonTest(leaderROI_contour,contourStruct.center,false) < 0 )
                continue;
            /// Uncomment to show/highlight blobs that appear inside ROI
#if (!ARM)
            cv::circle(frame,contourStruct.center, 6, cv::Scalar(255,255,255), 2);
#endif
        }

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
            unsigned int R,G,B;
            Mat RGB(1,1,CV_32FC3);
            Mat HSV(1,1,CV_32FC3);
            unsigned int n=0;
            R = G = B = 0;

            // get the bounding rectangle
            Rect roi = boundingRect(contours[contourIdx]);

            // iterate over bounding rectangle
            for (int y=roi.y; y<roi.y + roi.height; y++){
                for(int x=roi.x; x< roi.x + roi.width; x++){
                    // Check if point lies inside contour
                    if (pointPolygonTest(contours[contourIdx], Point2f(x,y),false) >= 0){
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
                        proximity tmpProx;
                            tmpProx.center = contourStruct.center;
                            tmpProx.radius = radius;
                        proximityVec[i].push_back(tmpProx);
                    }
                }

            }

        }

        contourStructVec.push_back(contourStruct);
    }


    if (!havePreviousState && params.sortByColor)
    {
        sort(contourStructVec.begin(), contourStructVec.end(), LED_Detector::sortByColor);

        // populate the vector of image points from the contourStructVec
        int stopHere = (contourStructVec.size() > params.maxBlobs) ? params.maxBlobs : contourStructVec.size();
        for (int i=0; i<stopHere; i++)
        {
            leds.push_back(contourStructVec[i].center);
        }
    }

    /// This is the beginning of modified code to selectively select "best" image points
    /// and handle duplicates
    if (havePreviousState)
    {
        // Flatten vector to more easily access elements for duplicate detection
        // (don't have to traverse across two dimensions)
        vector<proximity*> flat; // "flattened" vector (points to elements of proximityVec)
        for (int i=0; i<NO_LEDS; i++)
        {
            for (int j=0; j<proximityVec[i].size(); j++)
            {
                flat.push_back(&proximityVec[i][j]);
            }
        }

        // Detect and "group" duplicates
        vector<vector<proximity*> > groups;
        for (int i=0; i<flat.size(); i++)
        {
            if (flat[i]->isDuplicate > 0)
                continue;

            vector<proximity*> subgroup;
            subgroup.push_back(flat[i]);
            for (int j=i+1; j<flat.size(); j++)
            {
                if (samePoint(*flat[i], *flat[j]))
                {
                    flat[i]->isDuplicate = true;
                    flat[j]->isDuplicate = true;
                    subgroup.push_back(flat[j]);
                }
            }
            if (subgroup.size() > 1)
                groups.push_back(subgroup);
        }


        // Sort subgroups by (increasing) radius and keep only the one with the smallest radius
        for (int i=0; i<groups.size(); i++)
            sort(groups[i].begin(),groups[i].end(),compByRadius);


        // Eliminate any duplicate points (choosing to keep the one(s) with the lowest radius)
        n_detected = NO_LEDS;
        for (int i=0; i<groups.size(); i++)
        {
            for (int j=1; j<groups[i].size(); j++) // erase everything after the first element
                groups[i][j]-> delete_me = true;  // set to be deleted
        }
        for (int i=0; i<proximityVec.size(); i++)
        {
            for (int j=0; j<proximityVec[i].size(); j++)
            {
                if (proximityVec[i][j].delete_me)
                    proximityVec[i].erase(proximityVec[i].begin() + j);
            }

            // If any rows are empty, subtract from the detected LED count
            if (proximityVec[i].empty())
                n_detected--;
        }

        // Assume we don't see the aircraft if we see fewer than 3 LEDs
        // and use normal detection logic
        if (n_detected < 3)
        {
            havePreviousState = false;
            goto not_detected_stop;     // go back to beginning with assumption that we don't have previous state
        }


        // Sort by radius
        for (int i=0; i<NO_LEDS; i++)
            sort(proximityVec[i].begin(),proximityVec[i].end());  // sort by increasing radius


        // Replace any "missing" points with reprojImgPts from last time
        for (int i=0; i<NO_LEDS; i++)
        {
            if (proximityVec[i].empty())
            {
                proximity tmp;
                tmp.center = reprojImgPts[i];
                proximityVec[i].push_back(tmp);
            }
        }


        // Return the first 5 points in order, then add points in successive layers
        // (selected randomly) until we reach maximum (N) points
        const int LEDs_max = 8;

        // Pass the first LEDs
        for (int i=0; i<NO_LEDS; i++)
            leds.push_back(proximityVec[i][0].center);


        // Go along successive layers to add "extra" points choosing which to pass by random
        vector<Point2f*> layer;
        int layer_no = 0;
        while (leds.size() < LEDs_max)
        {

            // Populate the next layer if we have extracted all elements from previous one
            if (layer.empty())
            {
                layer_no++;
                for (int i=0; i<NO_LEDS; i++)
                {
                    if (proximityVec[i].size() > layer_no)
                        layer.push_back(&proximityVec[i][layer_no].center);
                }
            }
            if (layer.empty())
                break;

            int randomPick = rand() % layer.size();
            leds.push_back(*layer[randomPick]);
            layer.erase(layer.begin() + randomPick);
        }

    }

    if (havePreviousState)
        return true;
    else
        return false;
}



