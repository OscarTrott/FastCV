/**
 *  @brief Class defintion for sobel filtering a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file sobelfilter.cpp
 *
 */

#include "sobelfilter.h"

#include <iostream>

 /**
  *  @brief Default constructor
  *
  */
SobelFilter::SobelFilter() : verticalFilter(3, 3, CV_32FC1), horizontalFilter(3, 3, CV_32FC1)
{
    resetFilters();
}

/**
 *  @brief Processes the image
 *
 *  @param[in,out] img The image to apply the filtering to
 */
bool SobelFilter::processImage(cv::Mat& img)
{
    return true;
}


void SobelFilter::resetFilters()
{
    const float32_t corners = 3.0F;
    const float32_t edges = 10.0F;

    verticalFilter.at<float32_t>(0, 0) = corners;
    verticalFilter.at<float32_t>(0, 1) = edges;
    verticalFilter.at<float32_t>(0, 2) = corners;
    verticalFilter.at<float32_t>(1, 0) = 0.0F;
    verticalFilter.at<float32_t>(1, 1) = 0.0F;
    verticalFilter.at<float32_t>(1, 2) = 0.0F;
    verticalFilter.at<float32_t>(2, 0) = -corners;
    verticalFilter.at<float32_t>(2, 1) = -edges;
    verticalFilter.at<float32_t>(2, 2) = -corners;

    horizontalFilter.at<float32_t>(0, 0) = corners;
    horizontalFilter.at<float32_t>(1, 0) = edges;
    horizontalFilter.at<float32_t>(2, 0) = corners;
    horizontalFilter.at<float32_t>(0, 1) = 0.0F;
    horizontalFilter.at<float32_t>(1, 1) = 0.0F;
    horizontalFilter.at<float32_t>(2, 1) = 0.0F;
    horizontalFilter.at<float32_t>(0, 2) = -corners;
    horizontalFilter.at<float32_t>(1, 2) = -edges;
    horizontalFilter.at<float32_t>(2, 2) = -corners;
}
