/**
 *  @brief Implementation class for finding feature points within an image (distorted or undistorted) through pure randomness
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file random_featuredetector.cpp
 *
 */

#include "harriscorner_featuredetector.h"
#include <iostream>
#include <fstream>

 /**
  *  @details Do nothing
  */
HarrisFeatureDetector::HarrisFeatureDetector()
{}

bool HarrisFeatureDetector::processFeatures(const cv::Mat& img, cv::Mat& contours)
{
    cv::String currDataWindowName = "Current processing";

    cv::Mat interimImg = cv::Mat(img.size(), CV_32FC1);
    img.convertTo(interimImg, CV_32FC1);

    // Apply a gaussian filter to the image
    ImageFilter filter = ImageFilter();

    filter.gaussianFilter<float32_t>(interimImg, 0.5F);

    // Apply sobel operator in x and y
    SobelFilter sobelFilter = SobelFilter();

    cv::Mat xyDerivative = cv::Mat(interimImg.size(), CV_32FC2);
    sobelFilter.sobelFilter<float32_t>(interimImg, xyDerivative);

    // Compute corner strength (harris value)
    cv::Mat harrisResponse = cv::Mat(img.size(), CV_32FC1);
    const float32_t windowSizeSigma = 1.0F;

    cv::Mat xyList[2];
    cv::split(xyDerivative, &xyList[0]);

    cv::Mat x2 = cv::Mat(img.size(), CV_32FC1);
    cv::Mat y2 = cv::Mat(img.size(), CV_32FC1);
    cv::Mat xy = cv::Mat(img.size(), CV_32FC1);

    cv::multiply(xyList[0], xyList[0], x2);
    cv::multiply(xyList[1], xyList[1], y2);
    cv::multiply(xyList[0], xyList[1], xy);

    filter.gaussianFilter<float32_t>(x2, windowSizeSigma);

    filter.gaussianFilter<float32_t>(y2, windowSizeSigma);

    filter.gaussianFilter<float32_t>(xy, windowSizeSigma);

    contours = xy.clone();

    cv::Mat x2y2 = cv::Mat(img.size(), CV_32FC1);
    cv::Mat xyxy = cv::Mat(img.size(), CV_32FC1);

    cv::multiply(x2, y2, x2y2);
    cv::multiply(xy, xy, xyxy);

    cv::Mat detM = cv::Mat(img.size(), CV_32FC1);

    cv::subtract(x2y2, xyxy, detM);

    const float32_t k = 0.04;

    cv::Mat x2AddY2 = cv::Mat(img.size(), CV_32FC1);
    
    cv::add(x2, y2, x2AddY2);

    cv::subtract(detM, k * x2AddY2, harrisResponse);

    filter.gaussianFilter<float32_t>(harrisResponse, 2.0F);

    cv::normalize(harrisResponse, harrisResponse, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // Choose local maxima within a window

    const cv::Size& imgSize = harrisResponse.size();
    const uint32_t harrisThreshold = 4;

    harrisResponse.convertTo(harrisResponse, CV_8UC1);
    harrisResponse.convertTo(interimImg, CV_32FC1);
    sobelFilter.sobelFilter<float32_t>(interimImg, xyDerivative);
    cv::split(xyDerivative, &xyList[0]);

    cv::Point point = cv::Point(0, 0);

    cv::namedWindow(currDataWindowName); // Create a window
    cv::imshow(currDataWindowName, harrisResponse); // Show our image inside the created window.
    cv::waitKey(0); // Wait for any keystroke in the window

    for (int y = 0; y < imgSize.height; y += 2)
    {
        for (int x = 0; x < imgSize.width; x += 2)
        {
            uint32_t xIdx = x;
            uint32_t yIdx = y;

            uint32_t prevXIdx = x;
            uint32_t prevYIdx = y;

            uint32_t pixelVal = harrisResponse.at<uint8_t>(y,x);
            uint32_t prevPixelVal = 0U;

            while (pixelVal > harrisThreshold && pixelVal > prevPixelVal) {
                prevPixelVal = pixelVal;

                prevXIdx = xIdx;
                prevYIdx = yIdx;

                xIdx += xyList[0].at<float32_t>(yIdx, xIdx) >= 0.0F ? 1 : -1;
                yIdx += xyList[1].at<float32_t>(yIdx, prevYIdx) >= 0.0F ? 1 : -1;

                pixelVal = harrisResponse.at<uint8_t>(yIdx, xIdx);
            }

            if (pixelVal > harrisThreshold)
            {
                point.x = xIdx;
                point.y = yIdx;
                const float32_t radius = 2*std::abs((int)(xIdx - x)) + std::abs((int)(yIdx - y));
                mFeatures.push_back(Feature(prevXIdx, prevYIdx));
                cv::circle(harrisResponse, point, radius, cv::Scalar(0), -1);
                cv::circle(xyList[0], point, radius, cv::Scalar(0), -1);
                cv::circle(xyList[1], point, radius, cv::Scalar(0), -1);
            }
        }
    }

    return true;
}

void HarrisFeatureDetector::getFeatures(FeatureList& outFeatures)
{
    outFeatures = mFeatures;
}

