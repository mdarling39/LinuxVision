
#include "ThresholdedKF.hpp"
#include <iostream>

using namespace std;

/// Sample data settings
const int fs = 30;  // Hz
const int n_outliers = 5;
const int n_repeated = 20;
const int dx_mean = 800;

ThresholdedKF KF;


int main()
{

    struct ThresholdedKF::param_t KF_params;
    KF_params.A = 1;
    KF_params.C = 1;
    KF_params.Q = 1e-3;
    KF_params.R = 0;
    KF_params.P0 = 0.01;
    KF_params.KF_thresh = 1e5;
    KF_params.timeout = 1000;

    KF.set_params(KF_params);

    cout << "It Works" << endl;

    return 0;
}
