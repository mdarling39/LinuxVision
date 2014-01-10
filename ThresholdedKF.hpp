#ifndef THRESHOLDEDKF_HPP_INCLUDED
#define THRESHOLDEDKF_HPP_INCLUDED

/*
*  This file implements a thersholded Kalman filter similar to the one used in Ting et. al.
*  ("A Kalman Filter for Robust Outlier Detection"). Extreme observations are disregarded
*  in the Kalman filter "correction" step if the Mahalonobis distance exceeds a predefined
*  threshold value. This particular implementation works on a state vector of size NDIM
*  and treats the state elements independently. (That is, they are filtered separately
*  using NDIM Kalman filters running in parallel. The threshold is compared to the norm of
*  the Mahalonobis distances. This approach avoids costly matrix inversion and simplifies
*  the programming.)
*/

#include <cmath>
#include <algorithm>
#include <time.h>
#include <iostream>

using namespace std;

#define NDIM 6
#define SMALL 1e-8

class ThresholdedKF
{
/// Variables
public:

struct param_t{
    double A;           // state transition matrix
    double C;           // observation matrix
    double Q;           // process noise covariance
    double R;           // measurement noise covariance
    double P0;          // initial state estimate error covariance
    double KF_thresh;   // Kalman filter threshold value
    int timeout;        // timeout until filter reset (ms)
} params;

private:

double x_m[NDIM];       // a posteriori state estimate at previous timestep
double x_pri[NDIM];     // a priori state estimate at current timestep
double x[NDIM];         // a posteriori state estimate at current timestep
double P_pri;           // a priori state estimate error covariance
double P;               // a posteriori state estimate error covariance
double K;               // Kalman gain
double z[NDIM];         // observation at current timestep
double d[NDIM];         // Mahalanobis distance (ref: "A Kalman Filter for Robust Outlier Detection", Ting, et. al.)
double S;               // a posteriori prediction covariance
double r[NDIM];
struct timespec tic, toc;
double t_elapsed;

/// Methods
public:

ThresholdedKF();
bool set_params(param_t params);        // sets the filter parameter structure
bool initialize(double measured[NDIM]); // initializes/resets the thresholded Kalman filter
bool predict(double measured[NDIM]);                         // performs prediction step of filter and computes Mahalanobis distance
bool correct();    // performs correction step of filter
void get_state(double estimate[NDIM]);
double dtime(struct timespec*, struct timespec*);
};

#endif //THRESHOLDEDKF_HPP_INCLUDED
