/*
 * PnPObj.cpp
 *
 *  Created on: Feb 18, 2013
 *      Author: michaeldarling
 */

#include "PnPObj.hpp"


// Default constructor
PnPObj::PnPObj() {

	PnPObj::is_current = false;

	// Initialize transformation matricies
	CV2B.create(3,3,CV_64F);
	CV2B.setTo(cv::Scalar(0));
	CV2B.at<double>(0,2) = 1.0;
	CV2B.at<double>(1,0) = 1.0;
	CV2B.at<double>(2,1) = 1.0;
	cv::transpose(CV2B,B2CV); 		// CV2B and B2CV convert between OpenCV

	// Initialize rvec_guess and tvec_guess
	// (assume directly behind by 100 inches)
	rvec_guess.create(3,1,CV_64F);
	tvec_guess.create(3,1,CV_64F);
	double initial_guess[6] = {0,0,100,0,0,0};
	PnPObj::setGuess(initial_guess);

	// Set rvec and tvec equal to guesses
	PnPObj::resetGuess();

	// Get rotation matrix for initial guess
	cv::Rodrigues(rvec,rotMat);

	// Compute Euler angles (and generate B_rotMat)
	B_tvec.create(3,1,CV_64F);
	PnPObj::computeEuler();

	// Initialize axesPoints vector
	axesPoints.push_back(cv::Point3f(0,0,0));
	axesPoints.push_back(cv::Point3f(AXES_LN,0,0));
	axesPoints.push_back(cv::Point3f(0,AXES_LN,0));
	axesPoints.push_back(cv::Point3f(0,0,AXES_LN));
}



// custom copy constructor
	PnPObj::PnPObj(const PnPObj &src){

		// same as assignment operator (deep copy)
		(*this) = src;

}



// custom assignment operator
PnPObj& PnPObj::operator=(const PnPObj &rhs){

	// protect against self-assignment
	if (this == &rhs)
		return *this;

	this->modelPoints = rhs.modelPoints;
	this->axesPoints = rhs.axesPoints;
	this->imagePoints = rhs.imagePoints;
	this->projImagePoints = rhs.projImagePoints;
	this->projAxesPoints = rhs.projAxesPoints;

	this->cameraMatrix = rhs.cameraMatrix.clone();
	this->distCoeffs = rhs.distCoeffs.clone();
	this->rvec = rhs.rvec.clone();
	this->tvec = rhs.tvec.clone();
	this->B_tvec = rhs.B_tvec.clone();
	this->rvec_guess = rhs.rvec_guess.clone();
	this->tvec_guess = rhs.tvec_guess.clone();
	this->rotMat = rhs.rotMat.clone();
	this->B_rotMat = rhs.B_rotMat.clone();
	this->CV2B = rhs.CV2B.clone();
	this->B2CV = rhs.B2CV.clone();

	this->phi = rhs.phi;
	this->theta = rhs.theta;
	this->psi = rhs.psi;
	this->scaledReprojErr = rhs.scaledReprojErr;

	this->is_current = rhs.is_current;

	return (*this);
}



//custom destructor
//PnPObj::~PnPObj(){}   // cv::Mat objets automatically released using static counter


// set guess
void PnPObj::setGuess(double arr[6]) {
	tvec_guess.at<double>(0,0) = arr[0]*IN2MM;
	tvec_guess.at<double>(1,0) = arr[1]*IN2MM;
	tvec_guess.at<double>(2,0) = arr[2]*IN2MM;
	rvec_guess.at<double>(0,0) = arr[3]*DEG2RAD;
	rvec_guess.at<double>(1,0) = arr[4]*DEG2RAD;
	rvec_guess.at<double>(2,0) = arr[5]*DEG2RAD;
}



// reset the initial guess
void PnPObj::resetGuess() {
	rvec = rvec_guess;
	tvec = tvec_guess;
}



// compute Euler angles and compute B_rotMat
void PnPObj::computeEuler() {
	cv::Rodrigues(rvec,rotMat);			// update rotation matrix
	B_rotMat = CV2B * rotMat * B2CV;	// change the rotation matrix from CV convention to RHS body frame

	// get the Euler angles
	phi = atan2(B_rotMat.at<double>(2,1),B_rotMat.at<double>(2,2));  // get (-pi < phi < phi)
	double cos_theta = B_rotMat.at<double>(2,1) / sin(phi);
	if ( cos_theta > 0) // see if ||theta|| < 90 deg
        theta = asin(-B_rotMat.at<double>(2,0));        // 1st or 4th quadrants: (-pi/2 < theta < pi/2)
    else
        theta = M_PI - asin(-B_rotMat.at<double>(2,0)); // 2nd or 3rd quadrants: (-pi < theta < -pi/2) or (pi/2 < theta < pi)
	psi = atan2(B_rotMat.at<double>(1,0), B_rotMat.at<double>(0,0));  // get (-pi < psi < pi)



	theta1 = asin(-B_rotMat.at<double>(2,0));
	theta2 = M_PI - theta1;
	theta = (abs(theta1) < abs(theta2)) ? theta1 : theta2;  // return the angle closest to 0 deg


	phi = asin(B_rotMat.at<double>(2,1) / cos(theta));
	B_tvec.at<double>(0,0) = tvec.at<double>(2,0);			// get the body translation vector (Works fine)
	B_tvec.at<double>(1,0) = tvec.at<double>(0,0);
	B_tvec.at<double>(2,0) = tvec.at<double>(1,0);
}



// set new Euler angles and recompute rotMat and rvec
void PnPObj::set_Euler(double phiIN, double thetaIN, double psiIN) {

	// set Euler angles
	phi = phiIN;
	theta = thetaIN;
	psi = psiIN;

	// reconstruct rotation matrix: from object frame to camera frame
	B_rotMat.at<double>(0,0) = cos(theta)*cos(psi);
	B_rotMat.at<double>(0,1) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
	B_rotMat.at<double>(0,2) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
	B_rotMat.at<double>(1,0) = cos(theta)*sin(psi);
	B_rotMat.at<double>(1,1) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
	B_rotMat.at<double>(1,2) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
	B_rotMat.at<double>(2,0) = -sin(theta);
	B_rotMat.at<double>(2,1) = sin(phi)*cos(theta);
	B_rotMat.at<double>(2,2) = cos(phi)*cos(theta);

	// recompute rvec from new rotation matrix
	rotMat = B2CV * B_rotMat * CV2B;	// convert body rotation matrix
	cv::Rodrigues(rotMat,rvec);			// to OpenCV rotation matrix
}



// load camera properties from XML/YML file
bool PnPObj::setCamProps(const std::string filename) {

	// Open xml file with camera properties
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;

#ifdef DEBUG_STDOUT
	// write to standard output
	std::cout << "\n   Camera properties read from:\n   " << filename << std::endl;
	std::cout << "   cameraMatrix:\n   " << cameraMatrix << std::endl;
	std::cout << "   distCoeffs:\n   " << distCoeffs << "\n" << std::endl;
#endif /* DEBUG_MODE */

	if (cameraMatrix.empty())
		std::cerr << "Camera Matrix is empty!" << std::endl;
	else if (distCoeffs.empty())
		std::cerr << "Distortion Coefficients are empty" << std::endl;

	fs.release();
	return 1;
}



// set image points
void PnPObj::setImagePoints(std::vector<cv::Point2f>::iterator im_start,
		std::vector<cv::Point2f>::iterator im_end) {

	imagePoints.assign(im_start,im_end);

}



// get image points
std::vector<cv::Point2f> PnPObj::getImagePoints() {
	return imagePoints;
}



// swap image points (generates 5 sorted image points)
bool PnPObj::swapImagePoints(std::vector<cv::Point2f> imagePoints_IN, std::vector<cv::Point2f> &swappedImagePoints, const int num){

	if (imagePoints_IN.size() < (int)NO_LEDS) {
		std::cerr << "Not enough image points to swap" << std::endl;
		return false;
	}

	std::vector<cv::Point2f>::iterator last_pt = imagePoints_IN.begin() + (NO_LEDS-1);
    bool np1, np2, np3;
    np1 = np2 = np3 = false;
    if (imagePoints_IN.size() >= NO_LEDS+1){
        np1 = true;
        if (imagePoints_IN.size() >= NO_LEDS+2){
            np2 = true;
            if (imagePoints_IN.size() >= NO_LEDS+3){
                np3 = true;
            }
        }
    }

	switch (num){
	case 0:  // do nothing
		break;
	case 1:  // swap 5 & 6
		if (!np1)
			return false;
		std::iter_swap(last_pt,last_pt+1);
		break;
	case 2:  // swap 5 & 7
		if (!np2)
			return false;
		std::iter_swap(last_pt,last_pt+2);
		break;
	case 3:  // swap 4 & 6
		if (!np1)
			return false;
		std::iter_swap(last_pt-1,last_pt+1);
		break;
    case 4: // swap 3 & 6
        if (!np1)
            return false;
        std::iter_swap(last_pt-2,last_pt+1);
        break;
    case 5: // swap 2 & 6
        if (!np1)
            return false;
        std::iter_swap(last_pt-3,last_pt+1);
        break;
    case 6: // swap 1 & 6
        if (!np1)
            return false;
        std::iter_swap(last_pt-4,last_pt+1);
        break;
	case 7: // swap 4 & 7
		if (!np2)
			return false;
		std::iter_swap(last_pt-1,last_pt+2);
		break;
	case 8: // swap 5 & 8
		if (!np3)
			return false;
		std::iter_swap(last_pt,last_pt+3);
		break;
	case 9: // swap 4 & 8
		if (!np3)
			return false;
		std::iter_swap(last_pt-1,last_pt+3);
		break;
	default:
		std::cerr << "Currently don't support that many swaps of image points" << std::endl;
		break;
	};

	// only pass first 5 elements
	swappedImagePoints.assign(imagePoints_IN.begin(),imagePoints_IN.begin() + NO_LEDS);
	return true;
}


// set object points
bool PnPObj::setModelPoints(const char* pointsFilename) {
	std::ifstream pointsFile;
	pointsFile.open(pointsFilename);
	if (!pointsFile.is_open()) {
		std::cout << "Could not open points file" << std::endl;
		pointsFile.close();
		exit(-1);
	} else {
		float p1, p2, p3;
		while (pointsFile >> p1 >> p2 >> p3) {
			modelPoints.push_back(cv::Point3f(p1,p2,p3));
		}
	}
	pointsFile.close();

#ifdef DEBUG_STDOUT
	std::cout << "\n   Model points read from:\n   " << pointsFilename << std::endl;
	std::cout << "   modelPoints:\n   " << modelPoints << "\n" <<std::endl;
#endif /* DEBUG_MODE */

	return 1;
}



// solve PnP problem
void PnPObj::solve() {

	cv::solvePnP(modelPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, 1, CV_EPNP);

	// compute Euler angles and compute B_rotMat
	PnPObj::computeEuler();

	// reproject points and compute reprojection error
	PnPObj::projectAll();
	PnPObj::scaledReprojError();
}



// localize UAV by finding the best possible pose
// (returns 6-DOF state (dx,dy,dz,dphi,dtheta,dpsi) in units of (in,in,in,deg,deg,deg)
int PnPObj::localizeUAV(const std::vector<cv::Point2f> &imagePoints_IN, std::vector<double> &poseState, double &poseErr,
		const int max_swaps, const double perrTol, const double serrTol, const bool skipCorrelation) {

	// Create a backup of the last good PnP object. --> use this if no "good" pose is found
	PnPObj PnPObj_last_good(*this);


	// create memory buffers for holding past values  //TODO:  Consider putting these in a struct
	std::vector<PnPObj> PnPObj_buff;
	std::vector< std::vector<double> > poseState_buff;
	std::vector<double> error_buff;

	int iters = 0;

	if (imagePoints_IN.size() >= (int)NO_LEDS){ // have enough image points

		for (int i=0; i<3; i++){
			for (int j=0; j<max_swaps; j++){
				iters++; // advance iteration count

				// swap image points
				if(!PnPObj::swapImagePoints(imagePoints_IN,imagePoints,j)){
					// don't have enough blobs to sort using code j
					continue;
				}

				// don't need to call setImagePoints() here since having
				// swapImagePoints output to PnPObj::imagePoints member

				// loop through 3 possible re-correlation schemes between 3-D geometry and image points
				if (!skipCorrelation)
                    PnPObj::correlatePoints(i);

				PnPObj::solve();	 // WARNING:  Modifies private member function values of PnPObj!

                // Do not allow inverted flight ( |theta | > 90 deg)
                if (abs(poseState[1]) > M_PI )
                    scaledReprojErr = INFINITY;

				// Relax the tolerance if we have a previous frame to help us out
				if (scaledReprojErr < perrTol || (scaledReprojErr < serrTol && skipCorrelation)){
					poseErr = scaledReprojErr;
					poseState = PnPObj::getState();

					// convert units
					poseState[0] *= MM2IN;
					poseState[1] *= MM2IN;
					poseState[2] *= MM2IN;
					poseState[3] *= RAD2DEG;
					poseState[4] *= RAD2DEG;
					poseState[5] *= RAD2DEG;
					return iters;

				} else if (scaledReprojErr < serrTol) {
					// append to buffer and continue
					poseState_buff.push_back(PnPObj::getState());
					error_buff.push_back(scaledReprojErr);
					PnPObj_buff.push_back(*this); // append this instance of PnPObj to buffer
				}
//                if (skipCorrelation)  //only go through one iteration if we are providing image pts in order
//                    break;
			}
		}


		if (!error_buff.empty()) { // have a plausible estimate
			// find the minimum error in the buffer
			// TODO:  Can reduce memory requirements by defining a sorting algorithm for PnPObj_buff object
			int min_idx = std::distance(error_buff.begin(),std::min_element(error_buff.begin(),error_buff.end()));
			poseState = poseState_buff.at(min_idx);
			poseErr = error_buff.at(min_idx);
			(*this) = PnPObj_buff.at(min_idx);  // Restore the PnP object that minimizes error

			// convert units
			poseState[0] *= MM2IN;
			poseState[1] *= MM2IN;
			poseState[2] *= MM2IN;
			poseState[3] *= RAD2DEG;
			poseState[4] *= RAD2DEG;
			poseState[5] *= RAD2DEG;
			return iters;
		}
	}

	// Restore PnP object from backup and make the state NaN
	(*this) = PnPObj_last_good;
	fill(poseState.begin(),poseState.begin()+6,NAN);
	poseErr = NAN;
	return -1;
}



// project all points
void PnPObj::projectAll() {

	cv::projectPoints(modelPoints, rvec, tvec, cameraMatrix, distCoeffs, projImagePoints);
	cv::projectPoints(axesPoints, rvec, tvec, cameraMatrix, distCoeffs, projAxesPoints);

}



// compute scaled reprojection error
void PnPObj::scaledReprojError() {
	// Computes the scaled reprojection error.  Equal to the sum of the distances between the image
	// points and the reprojected image points divided by the maximum distance between image points

	// Sum up the shortest-distance between image points and reprojected image points
	double diff[NO_LEDS], sum;
	for (int i=0; i<NO_LEDS; i++) {
		diff[i] = cv::norm(imagePoints[i] - projImagePoints[i]);
	}
	sum = std::accumulate(diff, diff+NO_LEDS, 0.0);

	// compute the distance between all combinations of points, and find the maximum
	cv::Mat distances(NO_LEDS,NO_LEDS,CV_64F,cv::Scalar(0));
	for (int i = 0; i < NO_LEDS; i++) {
		for (int j = 0; j < NO_LEDS; j++){
			if (i <= j) continue;
			distances.at<double>(i,j) = cv::norm(imagePoints[i] - imagePoints[j]);
		}
	}

	double maxVal = *std::max_element(distances.begin<double>(),distances.end<double>());
	scaledReprojErr = (sum)/(maxVal);

}



// get scaledReprojError
double PnPObj::getScaledReprojError() {
	return scaledReprojErr;
}



// get state
std::vector<double> PnPObj::getState() {
	std::vector<double> state(6);
	state[0] = tvec.at<double>(2,0);
	state[1] = tvec.at<double>(0,0);
	state[2] = tvec.at<double>(1,0);;
	state[3] = phi;
	state[4] = theta;
	state[5] = psi;

	PnPObj::is_current = true;
	return state;
}



// draw over frame
void PnPObj::drawOverFrame(cv::Mat &src) {

    std::vector<double> state(6);
    state[0] = B_tvec.at<double>(0,0)*MM2IN;
    state[1] = B_tvec.at<double>(1,0)*MM2IN;
    state[2] = B_tvec.at<double>(2,0)*MM2IN;
    state[3] = phi*RAD2DEG;
    state[4] = theta*RAD2DEG;
    state[5] = psi*RAD2DEG;

    std::vector<cv::Point2f> projAxesPointsIN, projImagePointsIN;
    projAxesPointsIN = projAxesPoints;
    projImagePointsIN = projImagePoints;

    drawOverFrame(src, projAxesPointsIN, projImagePointsIN, state);
}

void PnPObj::drawOverFrame(cv::Mat &src, std::vector<double> stateIN) {
    /*
    HAVE TO RECOMPUTE REPROJECTION FOR stateIN (WHICH IS TYPICALLY A KALMAN FILTERED STATE)

    /// COPIED CODE FROM FUNCTIONS ABOVE (USED TO REVERSE-ENGINEER THIS CODE)

    B_tvec.at<double>(0,0) = tvec.at<double>(2,0);			// get the body translation vector (Works fine)
	B_tvec.at<double>(1,0) = tvec.at<double>(0,0);
	B_tvec.at<double>(2,0) = tvec.at<double>(1,0);

	// reconstruct rotation matrix: from object frame to camera frame
	B_rotMat.at<double>(0,0) = cos(theta)*cos(psi);
	B_rotMat.at<double>(0,1) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
	B_rotMat.at<double>(0,2) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
	B_rotMat.at<double>(1,0) = cos(theta)*sin(psi);
	B_rotMat.at<double>(1,1) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
	B_rotMat.at<double>(1,2) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
	B_rotMat.at<double>(2,0) = -sin(theta);
	B_rotMat.at<double>(2,1) = sin(phi)*cos(theta);
	B_rotMat.at<double>(2,2) = cos(phi)*cos(theta);

	// recompute rvec from new rotation matrix
	rotMat = B2CV * B_rotMat * CV2B;	// convert body rotation matrix
	cv::Rodrigues(rotMat,rvec);			// to OpenCV rotation matrix
    */

    // create new variables with "d_" prefix with same type and size as
    // PnPObj class members
    cv::Mat d_tvec(tvec.size(), tvec.type());
    cv::Mat d_rvec(rvec.size(), rvec.type());
    cv::Mat d_B_rotMat(B_rotMat.size(), B_rotMat.type());
    cv::Mat d_rotMat(rotMat.size(), rotMat.type());
    double d_phi, d_theta, d_psi;

    // Load d_tvec and d_rvec from the input state (making sure to convert units)
    d_tvec.at<double>(2,0) = stateIN[0] / MM2IN;  // dz
    d_tvec.at<double>(0,0) = stateIN[1] / MM2IN;  // dx
    d_tvec.at<double>(1,0) = stateIN[2] / MM2IN;  // dy
    d_phi =     stateIN[3] / RAD2DEG;             // dphi
    d_theta =   stateIN[4] / RAD2DEG;             // dtheta
    d_psi =     stateIN[5] / RAD2DEG;             // dpsi

    // Compute rotation matrix by copying code from set_Euler and replacing variables with
    // "d_" prefixed replacements
    d_B_rotMat.at<double>(0,0) = cos(d_theta)*cos(d_psi);
	d_B_rotMat.at<double>(0,1) = sin(d_phi)*sin(d_theta)*cos(d_psi) - cos(d_phi)*sin(d_psi);
	d_B_rotMat.at<double>(0,2) = cos(d_phi)*sin(d_theta)*cos(d_psi) + sin(d_phi)*sin(d_psi);
	d_B_rotMat.at<double>(1,0) = cos(d_theta)*sin(d_psi);
	d_B_rotMat.at<double>(1,1) = sin(d_phi)*sin(d_theta)*sin(d_psi) + cos(d_phi)*cos(d_psi);
	d_B_rotMat.at<double>(1,2) = cos(d_phi)*sin(d_theta)*sin(d_psi) - sin(d_phi)*cos(d_psi);
	d_B_rotMat.at<double>(2,0) = -sin(d_theta);
	d_B_rotMat.at<double>(2,1) = sin(d_phi)*cos(d_theta);
	d_B_rotMat.at<double>(2,2) = cos(d_phi)*cos(d_theta);

	// Compute rotation vector using "d_" prefixed variables
	d_rotMat = B2CV * d_B_rotMat * CV2B;
	cv::Rodrigues(d_rotMat,d_rvec);


    // Reproject points simlar to projectAll() method
    std::vector<cv::Point2f> projImagePointsIN, projAxesPointsIN;
    cv::projectPoints(modelPoints, rvec, tvec, cameraMatrix, distCoeffs, projImagePointsIN);
	cv::projectPoints(axesPoints, rvec, tvec, cameraMatrix, distCoeffs, projAxesPointsIN);

    // Draw over frame using thets projected values
    drawOverFrame(src, projAxesPointsIN, projImagePointsIN, stateIN);


}

void PnPObj::drawOverFrame(cv::Mat &src, std::vector<cv::Point2f> &projAxesPointsIN,
    std::vector<cv::Point2f> &projImagePointsIN, std::vector<double> stateIN)
{
	char pointLabel[5];
	cv::Point2f pointLabelLoc;
	char stateText1[100], stateText2[100], errText[100];

	if ((int)imagePoints.size() == NO_LEDS) {
		for (int i=0; i<NO_LEDS; i++) {
			cv::circle(src,imagePoints[i], 5, cv::Scalar(0,0,255), -1);
			sprintf(pointLabel,"%d",i+1);
			pointLabelLoc = imagePoints[i] - cv::Point2f(8,8);
			putText(src, pointLabel, pointLabelLoc, cv::FONT_HERSHEY_PLAIN,
					1.5, cv::Scalar(255,255,255), 1);


			cv::circle(src,projImagePointsIN[i], 3, cv::Scalar(0,255,255), 3);
			cv::line(src,projAxesPointsIN[0], projAxesPointsIN[3], cv::Scalar(255,0,0), 2);
			cv::line(src,projAxesPointsIN[0], projAxesPointsIN[1], cv::Scalar(0,0,255), 2);
			cv::line(src,projAxesPointsIN[0], projAxesPointsIN[2], cv::Scalar(0,255,0), 2);

			sprintf(stateText1," dx: %8.2f    dy: %8.2f   dz: %8.2f",
                stateIN[0],
                stateIN[1],
                stateIN[2]);

			sprintf(stateText2,"phi: %8.2f  theta: %8.2f  psi: %8.2f",
                stateIN[3],
                stateIN[4],
                stateIN[5]);

			if (is_current)
                sprintf(errText,"Reproj. Err: %10.6f", scaledReprojErr);
            else
                sprintf(errText,"Reproj. Err: %10.6f", NAN);
		}
	} else {
		sprintf(stateText1," dx: %8.2f    dy: %8.2f   dz: %8.2f",
				NAN, NAN, NAN);
		sprintf(stateText2,"phi: %8.2f  theta: %8.2f  psi: %8.2f", NAN, NAN, NAN);
		sprintf(errText,"Reproj. Err: %10.6f", NAN);
	}

	// Print state info
	cv::Point2f textPt1(30, src.rows-40);
	cv::Point2f textPt2(30, src.rows-20);
	cv::Point2f textPt3(30,src.rows-60);

	// TODO: Create a ROI with white space, add to image to make a washed out box, and put black text over it.
	putText(src, stateText1, textPt1, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255), 1.2);
	putText(src, stateText2, textPt2, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255), 1.2);
	putText(src, errText, textPt3, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255), 1.2);
}



// correlate imagePoints to modelPoints
void PnPObj::correlatePoints(int callNo) {
	// Assumes 5 LEDs numbered 1-5:
	// Order Found	LED ID #		Description
	// point1 	= 	[0] or [4] 		First wingtip found
	// point2 	= 	[1] or [3] 		First horizontal found (same side as point1)
	// point3 	= 	[4] or [0] 		Opposite wing of point1
	// point4 	= 	[3] or [1] 		Horizontal opposite side of point1
	// point5 	= 	[2] 			Vertical tail


	if ((int)imagePoints.size() > (int)NO_LEDS) {
		std::cerr << "Too many image points to correlate" << std::endl;
		exit(1);
	} else if ((int)imagePoints.size() < (int)NO_LEDS) {
		std::cerr << "Not enough image points to correlate" << std::endl;
		exit(1);
	}


	// create temporary variables to hold points while things get moved around
	std::vector<cv::Point2f> bufferImagePoints = imagePoints, newImagePoints = imagePoints;

	if(callNo > 2) std::cout << "ledFinder only supports a maximum of 3 calls!" << std::endl;

	// Compute the mean pixel location of the LEDs
	cv::Point2f zero(0.0f, 0.0f);
	cv::Point2f sum  = std::accumulate(bufferImagePoints.begin(), bufferImagePoints.end(), zero);
	cv::Point2f meanPoint(sum.x / bufferImagePoints.size(), sum.y / bufferImagePoints.size());

	// Compute distances from mean point and find index of farthest point
	int point1 = 0;
	double distFromMean, maxDistFromMean = 0;
	for (int i=0; i<NO_LEDS; i++) {
		distFromMean = cv::norm(cv::Mat(bufferImagePoints[i]) - cv::Mat(meanPoint));

		point1 = (distFromMean > maxDistFromMean) ? i : point1;
		maxDistFromMean = (distFromMean > maxDistFromMean) ? distFromMean : maxDistFromMean;
	}

	// Determine if this is a right or left wingtip
	char firstWing;
	if (bufferImagePoints[point1].x < meanPoint.x) {
		// left wingtip
		newImagePoints[0] = bufferImagePoints[point1];
		firstWing = 'L';
	} else {
		// right wingtip
		newImagePoints[4] = bufferImagePoints[point1];
		firstWing = 'R';
	}

	// Find the closest and farthest points from first wingtip and make them the
	// horizontal of the same side and opposite wingtip, respectively
	int point2 = 0, point3;
	double distFrom1, minDistFrom1 = INFINITY;
	std::vector<std::pair<double,int> > point3Pairs;

	for (int i=0; i<NO_LEDS; i++) {
		if (i==point1) continue; // Can't reuse points

		// find point2 (horizontal on same side as point1 wingtip)
		distFrom1 = cv::norm(cv::Mat(bufferImagePoints[i]) - cv::Mat(bufferImagePoints[point1]));
		point2 = (distFrom1 < minDistFrom1) ? i : point2;
		minDistFrom1 = (distFrom1 < minDistFrom1) ? distFrom1 : minDistFrom1;


		// find point3 (wingtip opposite of point1)
		// create a vector of distances from point 1 that can be sorted later
		point3Pairs.push_back(std::make_pair(i,distFrom1));
	}
	// sort by descending distance from point 1
	std::sort(point3Pairs.begin(), point3Pairs.end(), PnPObj::pairComparator);

	if (callNo==0) {
		// farthest point
		point3 = point3Pairs[0].first;
	} else if (callNo==1) {
		// second farthest point
		point3 = point3Pairs[1].first;
	} else if (callNo==2) {
		// third farthest point
		point3 = point3Pairs[2].first;
	}
	switch (firstWing) {
	case 'L':
		newImagePoints[1] = bufferImagePoints[point2];
		newImagePoints[4] = bufferImagePoints[point3];
		break;
	case 'R':
		newImagePoints[3] = bufferImagePoints[point2];
		newImagePoints[0] = bufferImagePoints[point3];
		break;
	default:
		std::cout << "Error evaluating if the first LED is left or right wingtip" << std::endl;
		break;
	}

	// Compute the angle of the wings and find which point (point4), when matched with the
	// known horizontal, most closely matches the wing angle.
	double wingSlope, wingAngle;
	wingSlope = (newImagePoints[4].y - newImagePoints[0].y)/(newImagePoints[4].x - newImagePoints[0].x);
	wingAngle = atan(wingSlope);

	int point4 = 0;
	double slope, angle, absAngleDiff, minAbsAngleDiff = INFINITY;
	for (int i=0; i<NO_LEDS; i++) {
		if (i==point1 || i==point2 || i== point3) continue;  // Can't reuse previous points
		switch (firstWing) {
		case 'L':
			slope = (newImagePoints[1].y - bufferImagePoints[i].y) / (newImagePoints[1].x - bufferImagePoints[i].x);
			break;
		case 'R':
			slope = (bufferImagePoints[i].y - newImagePoints[3].y) / (bufferImagePoints[i].x - newImagePoints[3].x);
			break;
		default:
			std::cout << "Error evaluating if the first LED is left or right wingtip" << std::endl;
			break;
		}

		angle = atan(slope);
		absAngleDiff = std::abs(angle - wingAngle);

		point4 = (absAngleDiff < minAbsAngleDiff) ? i : point4;
		minAbsAngleDiff = (absAngleDiff < minAbsAngleDiff) ? absAngleDiff : minAbsAngleDiff;
	}

	switch (firstWing) {
	case 'L':
		newImagePoints[3] = bufferImagePoints[point4];
		break;
	case 'R':
		newImagePoints[1] = bufferImagePoints[point4];
		break;
	default:
		std::cout << "Error evaluating if the first LED is left or right wingtip" << std::endl;
		break;
	}


	// The final LED (point5) must be the tail
	int point5;
	for (int i=0; i<NO_LEDS; i++) {
		if (i==point1 || i ==point2 || i==point3 || i==point4) {
			continue;
		} else {
			point5 = i;
			break;
		}
	}
	newImagePoints[2] = bufferImagePoints[point5];

	imagePoints = newImagePoints;
}
