/**
 *  @brief Declaration class for finding feature points within an image (distorted or undistorted) through pure randomness
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file random_featuredetector.h
 *
 */

#include "ifeaturedetector.h"
#include <opencv2/opencv.hpp>

#ifndef RANDFEATUREDETECTOR
#define RANDFEATUREDETECTOR

 /**
  *    @class IFeatureDetector
  *
  *    @brief Class to define functions used to find features within an image
  */
class RandFeatureDetector : public IFeatureDetector {

public:

    /**
     *  @brief Default constructor
     *
     *  @detailed Do nothing
     */
    RandFeatureDetector();

    /**
     *  @brief Process an image to find features within it
     *
     *  @param[in] img A reference to an image to find features within
     *
     *  @return True if the processing succeeded, False otherwise
     */
    bool processFeatures(const cv::Mat& img);

    /**
     *  @brief Return the features found by processing the image
     *
     *  @param[out] A reference to a list of features to populate with the found features
     */
    void getFeatures(FeatureList& outFeatures);

};

#endif // RANDFEATUREDETECTOR
