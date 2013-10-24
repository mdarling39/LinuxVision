#ifndef MORPH_HPP_INCLUDED
#define MORPH_HPP_INCLUDED

/*  morph namespace wrapper for morphological operations to take advantage
 *  of NEON hardware optimization, when available
 */

#include <opencv2/opencv.hpp>

#ifdef __ARM_NEON__
#include "neon_instructions.h"
#endif

namespace morph
{

#ifdef __ARM_NEON__
    void grabch(cv::Mat &input, cv::Mat &output, unsigned int ch)
        {
            const int items = 3*input.rows*input.cols;
            uchar* input_data = input.ptr<uchar>(0);
            uchar* output_data = output.ptr<uchar>(0);
            vector_extractch_of_n (input_data, output_data, items, ch);
        }

    void threshold_binary(cv::Mat &input, cv::Mat &output, unsigned int thresh)
    {
        const int items = input.rows*input.cols;
        uchar* input_data = input.ptr<uchar>(0);
        uchar* output_data = output.ptr<uchar>(0);
        vector_binary_thresh_of_n (input_data, output_data, items, thresh);
    }

    void dilate(cv::Mat &input, cv::Mat &output, cv::Mat &kernel)
    {
        uchar* input_data = input.ptr<uchar>(0);
        uchar* output_data = output.ptr<uchar>(0);
        uchar* kernel_data = kernel.ptr<uchar>(0);
        vector_dilate(input_data, output_data, input.rows, input.cols, kernel_data, (int) kernel.rows/2);
    }
#else
    void grabch(cv::Mat &input, cv::Mat &output, unsigned int ch)
    {
        int mixch[2];
        mixch[0] = ch;
        mixch[1] = 0;

        cv::mixChannels(&input,1,&output,1,mixch,1);
    }

    void threshold_binary(cv::Mat &input, cv::Mat &output, unsigned int thresh)
    {
        cv::threshold(input,output,thresh,255,cv::THRESH_BINARY);
    }

    void dilate(cv::Mat &input, cv::Mat &output, const cv::Mat &kernel)
    {
        cv::dilate(input,output,kernel);
    }
#endif

}

#endif // MORPH_HPP_INCLUDED
