/**
 *  @brief Implementation class for finding feature points within an image (distorted or undistorted) through pure randomness
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file random_featuredetector.cpp
 *
 */

#include "random_featuredetector.h"

/**
 *  @details Do nothing
 */
RandFeatureDetector::RandFeatureDetector()
{}

bool RandFeatureDetector::processFeatures(const cv::Mat& img)
{
    const uint32_t featNum = std::rand() % maxFeatureListLength;

    Feature feat_temp = Feature(0U, 0U);

    for (uint32_t i = 0U; i < featNum; i++)
    {
        const uint32_t xLoc = rand() % img.size().width;
        const uint32_t yLoc = rand() % img.size().height;

        feat_temp.x = xLoc;
        feat_temp.y = yLoc;

        mFeatures.push_back(feat_temp);
    }

    return true;
}

void RandFeatureDetector::getFeatures(FeatureList& outFeatures)
{
    outFeatures = mFeatures;
}

