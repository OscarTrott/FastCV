#pragma once

#ifndef DISPLAYIMAGES
#define DISPLAYIMAGES

#include "opencv2/opencv.hpp"

namespace Display
{
    static uint8_t winIdx = 0U;

    static void showImg(const cv::Mat& image, const std::string& winName = "", const bool doWait = true)
    {
        // Get the window name
        std::string winNameOutput = winName;

        if (winNameOutput.size() == 0U)
        {
            winNameOutput = "Window: " + std::to_string(winIdx++);
        }

        // Copy the image data
        cv::Mat rendering = image.clone();

        // Get the max and minimum values in the matrix
        float64_t min, max = 0.0;

        cv::minMaxLoc(image, &min, &max);

        rendering *= 255 / max;

        // Convert the data if it is not already in 0-255 range
        rendering.convertTo(rendering, CV_8UC1, 255.0 / max);

        //resize(rendering, rendering, cv::Size(800, 800));//Size(640, 480));

        cv::namedWindow(winNameOutput);
        cv::imshow(winNameOutput, rendering);

        if (doWait)
            cv::waitKey(0);
    }

};

#endif // !DISPLAYIMAGES

