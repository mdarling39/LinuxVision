/*
 * Threshold.cpp
 *
 *  Created on: Feb 18, 2013
 *      Author: michaeldarling
 */

#include "Threshold.hpp"


// Default constructor
Threshold::Threshold() {

}



// Constructor with image specified
Threshold::Threshold(cv::Mat &src) {
	Threshold();    //FORLINUX
	Threshold::set_image(src);
}



// set a source image
void Threshold::set_image(cv::Mat &src) {
	img = src;
}



// set blob parameters
void Threshold::set_params(CustomBlobDetector::Params &paramObj) {
	blobParams = paramObj;
}



// detect blobs
void Threshold::detect_blobs() {

	// set up and create the detector using the parameters
	CustomBlobDetector blob_detector(blobParams);
	blob_detector.detect(img, keyPoints);

	// display results
	//std::cout << (double)keyPoints.size() << " points detected" << std::endl;

}



// get keypoints
cv::vector<cv::KeyPoint> Threshold::get_keypoints() {
	return keyPoints;
}



// get Point2f vector
cv::vector<cv::Point2f> Threshold::get_points() {

	// convert points to a Point2f vector
	cv::KeyPoint::convert(keyPoints, points2f);

	// display points
	//std::cout << points2f << std::endl;

	return points2f;
}



// get a "blobs" image for plotting
void Threshold::createBlobsImage(cv::Mat &blobs, cv::Scalar color) {

	// display and draw the x y coordinates of the keypoints:
	//cv::drawKeypoints(blobs, keyPoints, blobs, color, cv::DrawMatchesFlags::DEFAULT);

	cv::vector<cv::Point2f> blobPoints = Threshold::get_points();
	for(unsigned int i=0; i<blobPoints.size(); i++) {
		cv::circle(blobs, blobPoints[i], 5, color, 3);
	}
}

