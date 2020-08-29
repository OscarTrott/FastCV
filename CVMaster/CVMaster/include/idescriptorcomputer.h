/**
 *  @brief Abstract class for computing descriptors based on features within a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file irenderer.h
 *
 */

#include "typedefinitions.h"
#include "feature.h"
#include <opencv2/opencv.hpp>

#ifndef IDESCRIPTOR
#define IDESCRIPTOR

 /**
  *    @class IDescriptorComputer
  *
  *    @brief Class to define functions to compute descriptors 
  */
class IDescriptorComputer {

public:
    /**
     *  @brief Default constructor
     *
     *  @detailed Do nothing
     */
    IDescriptorComputer() {}

    /**
     *  @brief Using the feature locations within the image compute feature descriptors
     *
     *  @param[in] img The image to compute the descriptors from
     *  @param[in,out] features The list of features to compute/update descriptors for
     *
     *  @return A list of feature descriptors where the index a maps to the feature location a in the input feature list
     */
    virtual std::vector<int64_t> findFeatures(const cv::Mat& img, FeatureList& features) = 0;
    /**
     *  @brief Compute the disparity between two descriptors
     *
     *  @param[in] descriptor1 The first descriptor to compute the disparity relative against
     *  @param[in] descriptor2 The second descriptor to compute the disparity to
     *
     *  @return The disparity between the two (range 0:inf)
     */
    virtual float32_t computeDisparity(const int64_t descriptor1, const int64_t descriptor2) = 0;
};

#endif // IFEATUREDETECTOR
