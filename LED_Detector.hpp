#ifndef LED_DETECTOR_HPP_INCLUDED
#define LED_DETECTOR_HPP_INCLUDED

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class LED_Detector
{
public:

struct Params
{
    // Default initial values are defined here.
    // Values can be modified as needed using public access


    unsigned char w_Circularity;
    unsigned char w_InertiaRatio;
    unsigned char w_Convexity;
    unsigned char w_BlobColor;

    int maxPoints;
    float threshold;
    float minDistBetweenBlobs;

    bool filterByColor;
    unsigned char targetBlobColor;

    bool filterByArea;
    float minArea;
    float maxArea;

    bool filterByCircularity;
    float minCircularity;
    float maxCircularity;
    float targetCircularity;

    bool filterByInertia;
    float minInertiaRatio;
    float maxInertiaRatio;
    float targetInertiaRatio;

    bool filterByConvexity;
    float minConvexity;
    float maxConvexity;
    float targetConvexity;

    bool filterByError;
    float maxError;
};

protected:

    // structure objects to hold information about detected features

    struct LEDs {
        Point2d center[1024]; // statically allocate space for more points than we expect to ever detect
        unsigned int n_found;
    };

	struct contourStructObj {
		Point2d center;
        float color;
        float area;
        float circularity;
        float inertiaRatio;
        float convexity;
        float error;

		/// I might have messed this up -- worry about sorting later
		/*
		const bool operator < (const contourStructObj &contours) const {
			return (error < contours.error);
		}
		*/
	};

private:


public:
void findLEDs(const cv::Mat&, cv::Mat&, cv::Mat&, vector<Point2f>&, const Params&);


};
#endif // LED_DETECTOR_HPP_INCLUDED
