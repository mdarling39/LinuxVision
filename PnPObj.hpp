/*
 * PnPObj.h
 *
 *  Created on: Feb 18, 2013
 *      Author: michaeldarling
 */

#ifndef PNPOBJ
#define PNPOBJ


#include <iostream>
#include <string>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "Global.hpp"


class PnPObj {

	// 3-D model points (pre-ordered to CV-oriented frame) and axes points
	std::vector<cv::Point3f> modelPoints, axesPoints;

	// 2-D image points and reprojected image points
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point2f> projAxesPoints;

	// Camera parameters
	cv::Mat cameraMatrix, distCoeffs;

	// Filepaths to camera calibration XML/YML file
	// and 3-D object points text file
	//char* camDataFilename, objPointsFilename;

	// Rotation and Translation vectors used by solvePnP (and tvec in body coordinates)
	cv::Mat rvec, tvec, B_tvec;

	// Initial guesses for rvec and tvec
	cv::Mat rvec_guess, tvec_guess;

	// Rotation Matrix and CV-to-Body and Body-to-CV transformation matrices
	cv::Mat rotMat, B_rotMat, CV2B, B2CV;

	// Euler angles
	double phi, theta, psi;

	// scaled reprojection error
	double scaledReprojErr;

	static bool pairComparator(std::pair<double,int> a, std::pair<double,int> b) {return a.second > b.second;}

public:

	bool is_current;  // helper variable to use as a flag of whether or not the state is current
	std::vector<cv::Point2f> projImagePoints;


	// constructors
	PnPObj();
	PnPObj(const PnPObj &src);
	PnPObj& operator=(const PnPObj &rhs);

	//destructor
	//~PnPObj();  cv::Mat objects automatically released using static counter

	// set rvec and tvec guess
	void setGuess(double arr[6]);

	// reset rvec and tvec to guess
	void resetGuess();

	// compute Euler angles and compute B_rotMat
	void computeEuler();

	// recompute rotMat from Euler angles
	void set_Euler(double phiIN, double thetaIN, double psiIN);

	// load camera properties (read from XML/YML file)
	bool setCamProps(const std::string filename);

	// set image points (pass as variable)
	void setImagePoints(std::vector<cv::Point2f>::iterator im_start,
			std::vector<cv::Point2f>::iterator im_end);

	// get imagePoints
	std::vector<cv::Point2f> getImagePoints();

	// set object points (read from .txt file)
	bool setModelPoints(const char* pointsFilename);

	// solve PnP problem
	void solve();

	// localize UAV by finding best pose estimate
	int localizeUAV(const std::vector<cv::Point2f> &imagePoints_IN, std::vector<double> &poseState, double &poseErr,
			const int max_swaps, const double perrTol, const double serrTol, const bool skipCorrelation=false);

	// project all points
	void projectAll();

	// compute scaled reprojection error
	void scaledReprojError();

	//get scaledReprojError
	double getScaledReprojError();

	// get state
	std::vector<double> getState();

	// draw over frame
	void drawOverFrame(cv::Mat &src);

	// generates a vector of 5 imagePoints
	bool swapImagePoints(const std::vector<cv::Point2f> imagePoints_IN, std::vector<cv::Point2f> &swappedImagePoints, const int num);

	// correlate imagePoints to modelPoints
	void correlatePoints(int callNo=1);


};

#endif /* PNPOBJ */
