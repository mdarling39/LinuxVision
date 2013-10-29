
#include "customblobdetector.hpp"
#include <iterator>
#include <iostream>
#include <cstdio>


// EVERYTHING BELOW HERE IS ORIGINAL OPENCV CODE
//#define DEBUG_BLOB_DETECTOR
#ifdef DEBUG_BLOB_DETECTOR
#  include "opencv2/opencv_modules.hpp"
#  ifdef HAVE_OPENCV_HIGHGUI
#    include "opencv2/highgui/highgui.hpp"
#  else
#    undef DEBUG_BLOB_DETECTOR
#  endif
#endif

using namespace cv;

/*
 *  CustomBlobDetector
 */
CustomBlobDetector::Params::Params()
{

    w_Circularity = 255;
    w_InertiaRatio = 255;
    w_Convexity = 255;
    w_BlobColor = 255;

    maxPoints = 10;
    maxError = 10;
    thresholdStep = 10;
    minThreshold = 50;
    maxThreshold = 220;
    minRepeatability = 2;
    minDistBetweenBlobs = 10;

    filterByColor = true;
    targetBlobColor = 255;

    filterByArea = true;
    minArea = 25;
    maxArea = 5000;

    filterByCircularity = false;
    minCircularity = 0.8f;
    maxCircularity = std::numeric_limits<float>::max();
    targetCircularity = 1.0f;

    filterByInertia = true;
    //minInertiaRatio = 0.6;
    minInertiaRatio = 0.1f;
    maxInertiaRatio = std::numeric_limits<float>::max();
    targetInertiaRatio = 1.0f;

    filterByConvexity = true;
    //minConvexity = 0.8;
    minConvexity = 0.95f;
    maxConvexity = std::numeric_limits<float>::max();
    targetConvexity = 1.0f;

    filterByError = true;
}

void CustomBlobDetector::Params::read(const cv::FileNode& fn )
{

    maxPoints = (int)fn["maxPoints"];
    maxError = fn["maxError"];
    minArea = fn["minArea"];
    maxArea = fn["maxArea"];
    minCircularity = fn["minCircularity"];
    maxCircularity = fn["maxCircularity"];
    minInertiaRatio = fn["minInertiaRatio"];
    maxInertiaRatio = fn["maxInertiaRatio"];
    minConvexity = fn["minConvexity"];
    maxConvexity = fn["maxConvexity"];
    minThreshold = fn["minThreshold"];

    targetCircularity = fn["targetCircularity"];
    targetInertiaRatio = fn["targetInertiaRatio"];
    targetConvexity = fn["targetConvexity"];
    targetBlobColor = (uchar)(int)fn["targetBlobColor"];

	w_Circularity = (uchar)(int)fn["w_Circularity"];
    w_InertiaRatio = (uchar)(int)fn["w_InertiaRatio"];
    w_Convexity = (uchar)(int)fn["w_Convexity"];
    w_BlobColor = (uchar)(int)fn["w_BlobColor"];

    filterByError = (int)fn["filterByError"] != 0 ? true : false;
    filterByArea = (int)fn["filterByArea"] != 0 ? true : false;
    filterByColor = (int)fn["filterByColor"] != 0 ? true : false;
    filterByCircularity = (int)fn["filterByCircularity"] != 0 ? true : false;
    filterByInertia = (int)fn["filterByInertia"] != 0 ? true : false;
    filterByConvexity = (int)fn["filterByConvexity"] != 0 ? true : false;


    // Depreciated BlobParams
    /*
    thresholdStep = fn["thresholdStep"];
    maxThreshold = fn["maxThreshold"];
    minRepeatability = (size_t)(int)fn["minRepeatability"];
    minDistBetweenBlobs = fn["minDistBetweenBlobs"];
    */

}

void CustomBlobDetector::Params::write(cv::FileStorage& fs) const
{
    fs << "w_Circularity" << w_Circularity;
    fs << "w_InertiaRatio" << w_InertiaRatio;
    fs << "w_Convexity" << w_Convexity;
    fs << "w_BlobColor" << w_BlobColor;

    fs << "maxPoints"      << maxPoints;
    fs << "maxError"      << maxError;
    fs << "thresholdStep" << thresholdStep;
    fs << "minThreshold" << minThreshold;
    fs << "maxThreshold" << maxThreshold;

    fs << "minRepeatability" << (int)minRepeatability;
    fs << "minDistBetweenBlobs" << minDistBetweenBlobs;

    fs << "filterByColor" << (int)filterByColor;
    fs << "targetBlobColor" << (int)targetBlobColor;

    fs << "filterByArea" << (int)filterByArea;
    fs << "minArea" << minArea;
    fs << "maxArea" << maxArea;

    fs << "filterByCircularity" << (int)filterByCircularity;
    fs << "minCircularity" << minCircularity;
    fs << "maxCircularity" << maxCircularity;
    fs << "targetCircularity" << targetCircularity;

    fs << "filterByInertia" << (int)filterByInertia;
    fs << "minInertiaRatio" << minInertiaRatio;
    fs << "maxInertiaRatio" << maxInertiaRatio;
    fs << "targetInertiaRatio" << targetInertiaRatio;

    fs << "filterByConvexity" << (int)filterByConvexity;
    fs << "minConvexity" << minConvexity;
    fs << "maxConvexity" << maxConvexity;
    fs << "targetConvexity" << targetConvexity;

    fs << "filterByError" << (int)filterByError;
}

CustomBlobDetector::CustomBlobDetector(const CustomBlobDetector::Params &parameters) :
    params(parameters)
{
}

void CustomBlobDetector::read( const cv::FileNode& fn )
{
    params.read(fn);
}

void CustomBlobDetector::write( cv::FileStorage& fs ) const
{
    params.write(fs);
}

void CustomBlobDetector::findBlobs(const cv::Mat &image, const cv::Mat &binaryImage, vector<Center> &centers, vector<contourStructObj> &contourStructs) const
{
    //(void)image;
    centers.clear();
    contourStructs.clear();


    vector < vector<Point> > contours;
    Mat tmpBinaryImage = binaryImage.clone();
    //findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    findContours(tmpBinaryImage,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);  // Use CV_RETR_EXTERNAL -- don't return "holes"


#ifdef DEBUG_BLOB_DETECTOR
    Mat anImage(480,640,CV_8UC3);
    drawContours(anImage,contours,-1,Scalar(0,0,255));
    imshow("binarized", anImage);
    waitKey(1);

    //  Mat keypointsImage;
    //  cvtColor( binaryImage, keypointsImage, CV_GRAY2RGB );
    //
    //  Mat contoursImage;
    //  cvtColor( binaryImage, contoursImage, CV_GRAY2RGB );
    //  drawContours( contoursImage, contours, -1, Scalar(0,255,0) );
    //  imshow("contours", contoursImage );
#endif

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Center center;
        contourStructObj contourStruct(center, 0);
        double targetError, targetErrorDenom;
        targetError = 0;
        targetErrorDenom = 0;
        center.confidence = 1;


        Moments moms = moments(Mat(contours[contourIdx]));
        if (params.filterByArea)
        {
            double area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
            contourStruct.area = area;
        }

        if (params.filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = arcLength(Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            contourStruct.circularity = ratio;
            targetError += params.w_Circularity * fabs(ratio - params.targetCircularity);
            targetErrorDenom += params.w_Circularity;
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
        }

        if (params.filterByInertia)
        {
            double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
            const double eps = 1e-2;
            double ratio;
            if (denominator > eps)
            {
                double cosmin = (moms.mu20 - moms.mu02) / denominator;
                double sinmin = 2 * moms.mu11 / denominator;
                double cosmax = -cosmin;
                double sinmax = -sinmin;

                double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
                double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
                ratio = imin / imax;
            }
            else
            {
                ratio = 1;
            }
            contourStruct.inertiaRatio = ratio;
            targetError += params.w_InertiaRatio * fabs(ratio - params.targetInertiaRatio);
            targetErrorDenom += params.w_InertiaRatio;
            if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
                continue;

            center.confidence = ratio * ratio;
        }

        if (params.filterByConvexity)
        {
            vector < Point > hull;
            convexHull(Mat(contours[contourIdx]), hull);
            double area = contourArea(Mat(contours[contourIdx]));
            double hullArea = contourArea(Mat(hull));
            double ratio = area / hullArea;
            contourStruct.convexity = ratio;
            targetError += params.w_Convexity * fabs(ratio - params.targetConvexity);
            targetErrorDenom += params.w_Convexity;
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
        }

        center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (params.filterByColor)
        {
            if ((double)image.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)) <= params.minThreshold) continue;

            contourStruct.blobColor = (double)image.at<uchar> (cvRound(center.location.y), cvRound(center.location.x));
            targetError += params.w_BlobColor *
                           (fabs(params.targetBlobColor - image.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)))
                            / (255 - params.minThreshold));;
            targetErrorDenom += params.w_BlobColor;
        }

        contourStruct.imageX = center.location.x;
        contourStruct.imageY = center.location.y;

        //compute blob radius
        {
            vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                Point2d pt = contours[contourIdx][pointIdx];
                dists.push_back(norm(center.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        // compute weighted sum
        if (targetErrorDenom == 0) targetErrorDenom = 1;
        targetError /= targetErrorDenom;
        contourStruct.center = center;
        contourStruct.targetError = targetError;

        centers.push_back(center);
        contourStructs.push_back(contourStruct);

#ifdef DEBUG_BLOB_DETECTOR
        //    circle( keypointsImage, center.location, 1, Scalar(0,0,255), 1 );
#endif
    }

#ifdef DEBUG_BLOB_DETECTOR
    //  imshow("bk", keypointsImage );
    //  waitKey();
#endif
}






void CustomBlobDetector::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat&) const
{

    //TODO: support mask
    keypoints.clear();
    Mat grayscaleImage;
    if (image.channels() == 3)
        cvtColor(image, grayscaleImage, CV_BGR2GRAY);
    else
        grayscaleImage = image;


    Mat binarizedImage;
    threshold(grayscaleImage, binarizedImage, params.minThreshold, 255, THRESH_BINARY);

    vector < Center > curCenters;
    vector <contourStructObj> curContourStructs;
    findBlobs(grayscaleImage, binarizedImage, curCenters, curContourStructs);

#ifdef DEBUG_BLOB_DETECTOR
    /*
    Mat anImage(480,640,CV_8UC3);
    drawContours(anImage,curContourStructs,-1,Scalar(0,0,255));
    imshow("binarized", anImage);
    waitKey(1);
    */
#endif


    // sort everything in order of increasing error
    sort(curContourStructs.begin(),curContourStructs.end());


/// Write to file
#ifdef FOUNDBLOBS_TO_FILE
    // print blob data to file
    bFile = fopen(blobFilename,"a");

    if (curContourStructs.size() == 0)
    {
        fprintf(bFile,"%15s%15s%15s%15s%15s%15s%15s%15s%15s\n",
                "NaN",
                "NaN",
                "NaN",
                "NaN",
                "NaN",
                "NaN",
                "NaN",
                "NaN",
                "NaN");
    } else {
        for (int i = 0; i < min((int)curContourStructs.size(), params.maxPoints); i++)
        {
            if (!params.filterByError || curContourStructs[i].targetError < params.maxError)
            {
                //std::cout << "  " << curContourStructs[i].targetError;
                fprintf(bFile,"%15d%15.4f%15.4f%15.4f%15.4f%15.4f%15.4f%15.4f%15.4f\n",
                        i,
                        curContourStructs[i].targetError,
                        curContourStructs[i].circularity,
                        curContourStructs[i].inertiaRatio,
                        curContourStructs[i].convexity,
                        curContourStructs[i].blobColor,
                        curContourStructs[i].area,
                        curContourStructs[i].imageX,
                        curContourStructs[i].imageY);
                KeyPoint kpt((Point2d)curContourStructs[i].center.location, (float)(curContourStructs[i].center.radius));
                keypoints.push_back(kpt);
            }
        }
    }


    // close the file
    fclose(bFile);
#endif /* FOUNDBLOBS_TO_FILE */


    // convert centers to keyPoints (modified to use errors)
    if (curContourStructs.size() != 0)
    {
        for (int i = 0; i < min((int)curContourStructs.size(), params.maxPoints); i++)
        {
            if (!params.filterByError || curContourStructs[i].targetError < params.maxError)
            {
                KeyPoint kpt((Point2d)curContourStructs[i].center.location, (float)(curContourStructs[i].center.radius));
                keypoints.push_back(kpt);
            }
        }
    }


#ifdef DEBUG_BLOB_DETECTOR
    /*
    namedWindow("keypoints", CV_WINDOW_NORMAL);
    Mat outImg = image.clone();
    for(size_t i=0; i<keypoints.size(); i++)
    {
        circle(outImg, keypoints[i].pt, keypoints[i].size, Scalar(255, 0, 255), -1);
    }
    //drawKeypoints(image, keypoints, outImg);
    imshow("keypoints", outImg);
    waitKey();
    */
#endif
}

