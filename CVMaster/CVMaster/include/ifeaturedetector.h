/**
 *  @brief Abstract class for finding feature points within an image (distorted or undistorted)
 *  
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *  
 *  @file ifeaturedetector.h
 *
 */

#include "feature.h"
#include <opencv2/opencv.hpp>

#ifndef IFEATUREDETECTOR
#define IFEATUREDETECTOR

/**
 *    @class IFeatureDetector
 *
 *    @brief Class to define functions used to find features within an image
 */
class IFeatureDetector {
   
public:
    const static uint32_t maxFeatureListLength = 1024U;
    
    /**
     *  @brief Default constructor
     *
     *  @detailed Do nothing
     */
    IFeatureDetector() {}

    /**
     *  @brief Process an image to find features within it
     *
     *  @param[in] img A reference to an image to find features within
     *
     *  @return True if the processing succeeded, False otherwise
     */
    virtual bool processFeatures(const cv::Mat &img) = 0U;

    /**
     *  @brief Return the features found by processing the image
     *
     *  @param[out] A reference to a list of features to populate with the found features
     */
    virtual void getFeatures(FeatureList& outFeatures) = 0U;
    
protected:
    FeatureList mFeatures;

};

#endif // IFEATUREDETECTOR
