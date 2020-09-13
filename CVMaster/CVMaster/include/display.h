#pragma once

#ifndef DISPLAYIMAGES
#define DISPLAYIMAGES

#include "opencv2/opencv.hpp"

class Display
{
public:
    Display();

    void showImg(const cv::Mat& image, const std::string& winName = "");

private:
    uint16_t winIdx;
    const std::string defaultStringName;
    cv::Mat rendering;
};

#endif // !DISPLAYIMAGES

