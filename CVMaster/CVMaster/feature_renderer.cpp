/**
 *  @brief Class delcaration for rendering features on a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file feature_renderer.h
 *
 */

#include "feature_renderer.h"

bool FeatureRenderer::processImage(cv::Mat& img)
{

    return true;
}

bool FeatureRenderer::processImage(cv::Mat& img, const FeatureList& inFeatures)
{
    const cv::Scalar colour = cv::Scalar(255, 0, 0);
    const uint32_t radius = 5;

    for (uint32_t i = 0; i < inFeatures.size(); i++) {
        Feature feat = inFeatures[i];

        cv::circle(img, cv::Point(feat.x, feat.y), radius, colour);
    }

    return true;
}
