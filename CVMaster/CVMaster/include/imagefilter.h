/**
 *  @brief Class declaration for filtering a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file imagefilter.h
 *
 */

#include "irenderer.h"
#include "typedefinitions.h"

#ifndef IMAGEFILTER
#define IMAGEFILTER

 /**
  *    @class ImageFilter
  *
  *    @brief Class to define functions used to render elements on an image
  */
class ImageFilter : public IRenderer {

public:
    /**
     *  @brief Default constructor
     *
     */
    ImageFilter();

    /**
     *  @brief Processes the image
     *
     *  @param[in,out] img The image to apply the filtering to
     */
    bool processImage(cv::Mat& img);

    /**
     *  @brief Perform a gaussian filtering of the image
     *
     *  @param[in,out] img The image to filter
     *  @param[in] sd The standard deviation to apply when computing the gaussian function
     *
     *  @return True if the filtering was performed successfully, false otherwise
     */
    template <typename dType>
    static cv::Mat gaussianFilter(cv::Mat& img, const float32_t sd)
    {
        if (sd == 0)
        {
            return img;
        }

        cv::Point p1 = cv::Point();

        // Calculate a new gaussian matrix if one has not yet been computed for this standard deviation

        const float64_t pi = 2 * acos(0.0);

        const uint32_t binCount = static_cast<uint32_t>(6.0F * sd) % 2U == 0U ? static_cast<uint32_t>(6.0F * sd) + 1U : static_cast<uint32_t>(6.0F * sd);

        cv::Mat gaussianMatrix = cv::Mat(static_cast<uint32_t>(binCount), static_cast<uint32_t>(binCount), CV_32FC1);

        // Compute guassian matrix
        for (uint32_t y = 0; y < binCount; y++)
        {
            for (uint32_t x = 0; x < binCount; x++)
            {
                const int32_t dx = x - binCount / 2U;
                const int32_t dy = y - binCount / 2U;

                const float32_t val = std::exp(-((std::pow(dx, 2.0F) + std::pow(dy, 2.0F)) / (2.0F * std::powf(sd, 2.0F)))) * 1.0F / (2.0F * pi * std::powf(sd, 2.0));

                p1.x = x;
                p1.y = y;

                gaussianMatrix.at<float32_t>(p1) = val;
            }
        }

        const cv::Size& imSize = img.size();
        const cv::Size& gaussianSize = gaussianMatrix.size();

        cv::Mat outImg = cv::Mat(imSize, img.type());

        cv::Point p2 = cv::Point();

        for (uint32_t y = 0U; y < imSize.height; y++)
        {
            for (uint32_t x = 0; x < imSize.width; x++)
            {
                float32_t sumVal = 0.0F;

                for (uint32_t i = 0; i < gaussianSize.height; i++)
                {
                    for (uint32_t j = 0; j < gaussianSize.width; j++)
                    {
                        const uint32_t xIdx = x + j - gaussianSize.width / 2U;
                        const uint32_t yIdx = y + i - gaussianSize.height / 2U;

                        if (xIdx >= 0U && xIdx < imSize.width && yIdx >= 0U && yIdx < imSize.height)
                        {
                            p1.x = j;
                            p1.y = i;

                            p2.x = xIdx;
                            p2.y = yIdx;

                            sumVal += gaussianMatrix.at<float32_t>(p1) * static_cast<float32_t>(img.at<dType>(p2));
                        }
                    }
                }

                p2.x = x;
                p2.y = y;

                outImg.at<dType>(p2) = sumVal;
            }
        }

        return outImg;
    }
};

#endif // IMAGEFILTER
