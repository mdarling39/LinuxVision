#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <ctime>
#include "ThresholdedKF.hpp"

#define PI M_PI

using namespace std;

/// Sample data settings
const int fs = 20;    // Hz
const int n_outliers = 5;
const int n_repeated = 10;
const int mean[] = {800, 0, 0, 0, 0, 0};
const int amplitude[] = {50, 50, 50, 50, 50, 50};
const int freq_min = 3;
const int freq_max = 10;
const int t_start = 0;
const int t_end = 120;

ThresholdedKF KF;

double frand(double low, double high)
{
    return (high - low) * (double)rand()/(double)RAND_MAX + low;
}

int main()
{

    /// Generate data
    // seed random number generator
    srand( time(NULL) );

    // create t vector
    vector<double> t;
    t.push_back(t_start);
    while (t.back() < t_end)
    {
        t.push_back(t.back() + (1.0/fs));
    }

    // create *clean* [dx, dy, dz, droll, dpitch, dyaw] vector
    vector<vector<double> > data(6,vector<double>(t.size()));
    int nrows = data.size();
    int ncols = t.size();

    for (int i=0; i<nrows; i++)
    {
    double freq = frand(freq_min, freq_max);

        double offset = frand(0,360);
        for (int j=0; j<ncols; j++)
        {
            data[i][j] = mean[i] + amplitude[i]*sin((freq*t[j] + offset) * PI/180);
        }
    }


    // inject outliers  (NOT WORKING YET)
    for (int k=0; k<n_outliers; k++)
    {

        int outlier_begin = rand()%ncols;
        int start = outlier_begin;
        int stop = ((start + n_repeated) < ncols) ? start+n_repeated : ncols;

        for (int i=0; i<nrows; i++)
        {
            if (i == 0)
            {
            fill(data[i].begin()+start, data[i].begin()+stop,
                (2.0 * frand(0,1) - 1) * mean[i]/5.0 + mean[i]/2.0);
            } else {
            fill(data[i].begin()+start, data[i].begin()+stop,
                amplitude[i] * (2.0 * rand()/(double)RAND_MAX - 1));
            }
        }
    }


    // save data to file for plotting later
    ofstream myfile;
    myfile.open("data.txt");
    for (int j=1; j<ncols; j++)
    {
        myfile << t[j] << ",";
        for (int i=0; i<nrows; i++)
            myfile << data[i][j] << ",";
        myfile << "\n";
    }
    myfile.close();


    /// Filter data
    // choose filter parameters
    ThresholdedKF::param_t KF_parms;
    KF_parms.A = 1;
    KF_parms.C = 1;
    KF_parms.Q = 1e-3;
    KF_parms.R = 0;
    KF_parms.P0 = 0.01;
    KF_parms.KF_thresh = 1e5;
    KF_parms.timeout = 1000;


    // open file for saving to later
    ofstream myfile2;
    myfile2.open("filtered.txt");

    KF.set_params(KF_parms);


    // Create timer object to use for delay
    struct timespec tic, toc;
    clock_gettime(CLOCK_MONOTONIC, &tic);
    clock_gettime(CLOCK_MONOTONIC, &toc);
    double secs;

    for (int j=0; j<ncols; j++)
    {
        // Add a delay to make pseudo real-time
        do
        {
            clock_gettime(CLOCK_MONOTONIC,&toc);
            secs = toc.tv_sec - tic.tv_sec + (toc.tv_nsec - tic.tv_nsec)/1000000000.0;

        } while (secs < 1.0/fs);


        double estimate[6];
        double measurement[6];
        for (int k=0; k<nrows; k++)
            measurement[k] = data[k][j];

        if (j==0) // force a first filter initialization
            KF.initialize(measurement);
        KF.predict(measurement);
        KF.correct();
        KF.get_state(estimate);

        myfile2 << t[j] << ",";
        for (int i=0; i<nrows; i++)
            myfile2 << estimate[i] << ",";
        myfile2 << "\n";

        clock_gettime(CLOCK_MONOTONIC, &tic);
        printf("\r%5.2f secs",t[j]);
        fflush(stdout);
    }
    myfile2.close();

}
