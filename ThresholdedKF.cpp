# include "ThresholdedKF.hpp"

using namespace std;


ThresholdedKF::ThresholdedKF()
{
    // Default values
    params.A = 1;
    params.C = 1;
    params.Q = 1e-4;
    params.R = 1e-4;
    params.P0 = 0.01;
    params.KF_thresh = 1e4;
    params.timeout = 1000;

    // initialize timer
    clock_gettime(CLOCK_MONOTONIC, &tic);
    clock_gettime(CLOCK_MONOTONIC, &toc);
    t_elapsed = dtime(&tic, &toc);
}


bool ThresholdedKF::set_params(param_t new_params)
{
    params = new_params;
    return true;
}


bool ThresholdedKF::initialize(double measurement[NDIM])
{
    // Set/reset the Kalman filter
    copy(measurement, measurement+NDIM, z);
    copy(z, z+NDIM, x_m);
    P = params.P0;

    // re-initialize the timer
    clock_gettime(CLOCK_MONOTONIC, &tic);
    t_elapsed = 0;
    return true;
}




bool ThresholdedKF::forced_reset()
{
    // Set the elapsed time to inifinity so that filter will be re-initialized
    // in predict step
    t_elapsed = INFINITY;

    return true;
}



bool ThresholdedKF::predict(double measurement[NDIM])
{

    // re-initialize the filter if timeout has occurred
    if (t_elapsed >= (params.timeout/1000.0))
    {
        initialize(measurement);
    }

    copy(measurement, measurement+NDIM, z);

    // KF prediction
    for(int i=0; i<NDIM; i++)
        x_pri[i] = params.A * x_m[i];
    P_pri = params.A * P * params.A + params.Q;


    // compute norm of Mahalanobis distance
    S = params.C * P_pri * params.C + params.R;
    for(int i=0; i<NDIM; i++)
    {
        r[i] = z[i] - params.C * x_pri[i];
        d[i] = r[i] * 1/(S + SMALL) * r[i];
    }

    return true;
}


bool ThresholdedKF::correct()
{
    // compute norm of Mahalanobis distance
    double normVal=0;
    for(int i=0; i<NDIM; i++)
        normVal += d[i]*d[i];
    normVal = sqrt(normVal);

    bool rval;
    if(normVal < params.KF_thresh)
    {
        // perform correction step
        K = P * params.C * 1.0/(S + SMALL);
        for(int i=0; i<NDIM; i++)
            x[i] = x_pri[i] + K * r[i];
        P = P_pri - K * params.C * P_pri;

        // record the time
        clock_gettime(CLOCK_MONOTONIC, &tic);
        t_elapsed = 0;

        rval = true;
    }else{
        // don't perform correction step
        for (int i=0; i<NDIM; i++)
            x[i] = x_pri[i];
        P = P_pri;

        // update time since last successful update
        clock_gettime(CLOCK_MONOTONIC, &toc);
        t_elapsed = dtime(&tic, &toc);

        rval = false;
    }

    // Update x_m for next iteration
    for (int i=0; i<NDIM; i++)
        x_m[i] = x[i];

    return rval;
}


void ThresholdedKF::get_state(double estimate[NDIM])
{
    for (int i=0; i<NDIM; i++)
        estimate[i] = x[i];
}


double ThresholdedKF::dtime(struct timespec *start, struct timespec *finish)
{
    double elapsed = (finish->tv_sec - start->tv_sec);
    elapsed += (finish->tv_nsec - start->tv_nsec) / 1000000000.0;
    return elapsed;
}
