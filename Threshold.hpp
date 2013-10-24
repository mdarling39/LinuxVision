/*
 * Threshold.h
 *
 *  Created on: Feb 18, 2013
 *      Author: michaeldarling
 */

#ifndef THRESHOLD
#define THRESHOLD

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Global.hpp"
#include "customblobdetector.hpp"


class Threshold {

	// Point2f vector of keypoints
	std::vector<cv::Point2f> points2f;

	// keyPoints objects
	cv::vector<cv::KeyPoint> keyPoints;
	cv::vector< cv::vector <cv::Point> > contours;
	cv::vector< cv::vector <cv::Point> > approxContours;

	// blob parameters
	CustomBlobDetector::Params blobParams;

	// feature detector object
	cv::Ptr<cv::FeatureDetector> blob_detector;

	// source image and blobs-only image
	cv::Mat img, blobs;


public:

	// default constructor
	Threshold();
	Threshold(cv::Mat &src);

	// set a source image
	void set_image(cv::Mat &src);

	// set blob parameters
	void set_params(CustomBlobDetector::Params &paramObj);

	// detect blobs
	void detect_blobs();

	// get keypoints
	cv::vector<cv::KeyPoint> get_keypoints();

	// get Piont2f vector
	cv::vector<cv::Point2f> get_points();

	// get a "blobs" image for plotting
	void createBlobsImage(cv::Mat &blobs,  cv::Scalar color=cv::Scalar(0,255,0));

};



#endif /* THRESHOLD */
