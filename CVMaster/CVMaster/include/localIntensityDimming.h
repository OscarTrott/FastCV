

#pragma once

#ifndef LOCALINTENSITYDIMMING_H
#define LOCALINTENSITYDIMMING_H

#include "opencv2/opencv.hpp"
#include "typedefinitions.h"

class LocalDimming {
public:
    void processImage(cv::Mat& image, int16_t winSize)
    {
        cv::Mat outImage = image.clone();

        cv::Size imSize = image.size();

        cv::Point tempPoint;

        for (uint16_t y = 0U; y < imSize.height; y++)
        {
            for (uint16_t x = 0U; x < imSize.width; x++)
            {
                uint8_t maxVal = 0U;
                uint8_t minVal = 255U;

                for (int16_t dx = -winSize + 1; dx < winSize; dx++)
                {
                    for (int16_t dy = -winSize + 1; dy < winSize; dy++)
                    {
                        if (x + dx < 0 || y + dy < 0 || x + dx >= imSize.width || y + dy >= imSize.height)
                            continue;

                        tempPoint.x = x + dx;
                        tempPoint.y = y + dy;

                        uint8_t pixelVal = image.at<uint8_t>(tempPoint);

                        if (pixelVal > maxVal)
                            maxVal = pixelVal;
                        else if (pixelVal < minVal)
                            minVal = pixelVal;
                    }
                }

                uint8_t &pixelVal = outImage.at<uint8_t>(tempPoint);

                pixelVal = 255 * (pixelVal) / maxVal;
            }
        }

        image = outImage.clone();
    }
};

#endif // !LOCALINTENSITYDIMMING_H

