/**
 *  @brief Abstract class for rendering some information upon a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file irenderer.h
 *
 */

#include <opencv2/opencv.hpp>

#ifndef IRENDERER
#define IRENDERER

 /**
  *    @class IRenderer
  *
  *    @brief Class to define functions used to render elements on an image
  */
class IRenderer {

public:
    /**
     *  @brief Default constructor
     *
     *  @detailed Do nothing
     */
    IRenderer() {}

    /**
     *  @brief Processes the image in some way
     *
     *  @param[in,out] img The image to apply the rendering to
     */
    virtual bool processImage(cv::Mat& img) = 0U;
};

#endif // IFEATUREDETECTOR
