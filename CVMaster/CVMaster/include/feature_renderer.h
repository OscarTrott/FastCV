/**
 *  @brief Class delcaration for rendering features on a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file feature_renderer.h
 *
 */

#include <opencv2/opencv.hpp>
#include "ifeaturedetector.h"
#include "irenderer.h"

#ifndef FEATURERENDERER
#define FEATURERENDERER

 /**
  *    @class FeatureRenderer
  *
  *    @brief Class to define functions used to render features on an image
  */
class FeatureRenderer : public IRenderer{

public:

    /**
     *  @brief Default constructor
     *
     *  @detailed Do nothing
     */
    FeatureRenderer() {}

    /**
     *  @brief Processes the image in some way
     *
     *  @param[in,out] img The image to apply the rendering to
     */
    bool processImage(cv::Mat& img);

    /**
     *  @brief Processes the image in some way
     *
     *  @param[in,out] img The image to apply the rendering to
     */
    bool processImage(cv::Mat& img, const FeatureList& inFeatures);
};

#endif // IFEATUREDETECTOR
