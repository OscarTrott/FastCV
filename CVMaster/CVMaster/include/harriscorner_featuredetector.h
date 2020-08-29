/**
 *  @brief Declaration class for finding feature points within an image (distorted or undistorted) using Harris corner detector
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file harriscorner_featuredetector.h
 *
 */

#include "ifeaturedetector.h"
#include "imagefilter.h"
#include "sobelfilter.h"
#include <opencv2/opencv.hpp>

#ifndef HARRISFEATUREDETECTOR
#define HARRISFEATUREDETECTOR

 /**
  *    @class HarrisFeatureDetector
  *
  *    @brief Class to define functions used to find features within an image
  */
class HarrisFeatureDetector {

public:

    /**
     *  @brief Default constructor
     *
     *  @detailed Do nothing
     */
    HarrisFeatureDetector();

    /**
     *  @copydoc IFeatureDetector::processFeatures()
     */
    bool processFeatures(const cv::Mat& img, cv::Mat& contours);

    /**
     *  @copydoc IFeatureDetector::getFeatures()
     */
    void getFeatures(FeatureList& outFeatures);

private:
    FeatureList mFeatures;

};

#endif // RANDFEATUREDETECTOR
