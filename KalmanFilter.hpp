/*
 * KalmanFilter.h
 *
 *  Created on: Apr 6, 2013
 *      Author: michaeldarling
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <math.h>
#include "Global.hpp"

struct kFilterSt {
	int n_states, n_measurement, n_controls, type;
	cv::Point2f estimate;
};


#endif /* KALMANFILTER_H_ */
