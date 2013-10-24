
#ifndef __CUSTOMBLOBDETECTOR_HPP__
#define __CUSTOMBLOBDETECTOR_HPP__

#ifndef __OPENCV_PRECOMP_H__
#define __OPENCV_PRECOMP_H__

#ifdef HAVE_CVCONFIG_H
#include "cvconfig.h"
#endif


#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/internal.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <algorithm>
#include "Global.hpp"

// make sure fprintf can be used
#ifdef FOUNDBLOBS_TO_FiLE
#include <stdio.h>
#endif //FOUNDBLOBS_TO_FILE

#ifdef HAVE_TEGRA_OPTIMIZATION
#include "opencv2/features2d/features2d_tegra.hpp"
#endif

#endif

#ifdef __cplusplus
#include <limits>

using namespace cv;


class CV_EXPORTS_W CustomBlobDetector : public cv::FeatureDetector
{
public:
	struct CV_EXPORTS_W_SIMPLE Params
	{
		CV_WRAP Params();

		CV_PROP_RW uchar w_Circularity, w_InertiaRatio, w_Convexity, w_BlobColor;

		CV_PROP_RW int maxPoints;
		CV_PROP_RW float maxError;
		CV_PROP_RW float thresholdStep;
		CV_PROP_RW float minThreshold;
		CV_PROP_RW float maxThreshold;
		CV_PROP_RW size_t minRepeatability;
		CV_PROP_RW float minDistBetweenBlobs;

		CV_PROP_RW bool filterByColor;
		CV_PROP_RW uchar targetBlobColor;

		CV_PROP_RW bool filterByArea;
		CV_PROP_RW float minArea, maxArea;

		CV_PROP_RW bool filterByCircularity;
		CV_PROP_RW float minCircularity, maxCircularity, targetCircularity;

		CV_PROP_RW bool filterByInertia;
		CV_PROP_RW float minInertiaRatio, maxInertiaRatio, targetInertiaRatio;

		CV_PROP_RW bool filterByConvexity;
		CV_PROP_RW float minConvexity, maxConvexity, targetConvexity;

		CV_PROP_RW bool filterByError;

		void read( const FileNode& fn );
		void write( FileStorage& fs ) const;
	};

	CV_WRAP CustomBlobDetector(const CustomBlobDetector::Params &parameters = CustomBlobDetector::Params());

	virtual void read( const FileNode& fn );
	virtual void write( FileStorage& fs ) const;

protected:

	struct CV_EXPORTS Center
	{
		Point2d location;
		double radius;
		double confidence;
	};

	struct CV_EXPORTS contourStructObj {
		Center center;
		double targetError, area, circularity,
		inertiaRatio, convexity, blobColor,
		imageX, imageY;
		contourStructObj(Center a, double b) {
			center = a;
			targetError = b;
			area = 0;
			circularity = 0;
			inertiaRatio = 0;
			convexity = 0;
			blobColor = 0;
			imageX = 0;
			imageY = 0;
		}
		const bool operator < (const contourStructObj &a) const {
			return (targetError < a.targetError);
		}
	};


	virtual void detectImpl( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;
	virtual void findBlobs(const Mat &image, const Mat &binaryImage, vector<Center> &centers, vector<contourStructObj> &contourStructs) const;

	Params params;
	//AlgorithmInfo* info() const;
};


#endif /* __cplusplus */

#endif

/* End of file. */
