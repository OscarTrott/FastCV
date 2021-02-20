

#pragma once

#ifndef LOCALINTENSITYDIMMING_H
#define LOCALINTENSITYDIMMING_H

#include "opencv2/opencv.hpp"
#include "typedefinitions.h"

class LocalDimming {
public:
    template <typename imgType>
    cv::Mat processImage(const cv::Mat& image, const int16_t winSize)
    {
        cv::Mat outImage = image.clone();

        cv::Size imSize = image.size();

        cv::Point tempPoint;

        for (uint16_t y = 0U; y < imSize.height; y++)
        {
            for (uint16_t x = 0U; x < imSize.width; x++)
            {


            }
        }

        return outImage;
    }
};

#endif // !LOCALINTENSITYDIMMING_H

