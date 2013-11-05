#ifndef LED_DETECTOR_HPP_INCLUDED
#define LED_DETECTOR_HPP_INCLUDED

//#define LED_DETECTOR_DEBUG

#include "Global.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm> // sort, min
#include <stdio.h>

using namespace std;
using namespace cv;

class LED_Detector
{
public:

struct Params
{
    // Default initial values are defined here.
    // Values can be modified as needed using public access

    float threshold;
    int maxBlobs;

    bool sortByColor;
    bool filterByColor;
    float maxColor;     // maximum delta from target color
    float targetColor;

    bool filterByArea;
    float minArea;
    float maxArea;

    bool filterByCircularity;
    float minCircularity;
    float maxCircularity;
    float targetCircularity;

    bool filterByAspectRatio;
    float minAspectRatio;
    float maxAspectRatio;

    int localRadius;    // radius around previous point in percent span
};

    // structure objects to hold information about detected features

    struct LEDs {
        Point2d center[1024]; // statically allocate space for more points than we expect to ever detect
        unsigned int n_found;
    };

	struct contourStructObj {
		Point2f center;
		float radius;
        float color;
        float color_delta;
        float area;
        float circularity;
        float aspectRatio;
	};

	static bool sortByColor(const contourStructObj &a, const contourStructObj &b)
	{
        return a.color_delta < b.color_delta;
	}


private:

    struct proximity
    {
        Point2f center;
        float radius;
    };
    vector<proximity> proximityVec;
    float span;


public:
void findLEDs(const cv::Mat&, cv::Mat&, cv::Mat&, vector<Point2f>&, const Params&);
bool findLEDs(const cv::Mat&, cv::Mat&, cv::Mat&, vector<Point2f>&, const Params&,
    bool, const vector<Point2f>&);


};
#endif // LED_DETECTOR_HPP_INCLUDED
