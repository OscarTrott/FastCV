/**
 *  @brief Class declaration for sobel filtering a given image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file sobelfilter.h
 *
 */

#include "irenderer.h"
#include "typedefinitions.h"

#ifndef SOBELFILTER
#define SOBELFILTER

 /**
  *    @class SobelFilter
  *
  *    @brief Class to define functions used to estimate approximate intensity derivatives across the image
  */
class SobelFilter {

public:
    /**
     *  @brief Default constructor
     *
     */
    SobelFilter();

    /**
     *  @brief Processes the image
     *
     *  @param[in,out] img The image to apply the filtering to
     */
    bool processImage(cv::Mat& img);

    /**
     *  @brief Perform a sobel filtering of the image
     *
     *  @param[in,out] img The image to filter
     *
     *  @return True if the filtering was performed successfully, false otherwise
     */
    template <typename imgType>
    cv::Mat sobelFilter(cv::Mat img, const bool horizontal)
    {
        cv::Mat outImg = cv::Mat(img.size(), CV_8UC1);

        const cv::Size& imSize = img.size();

        cv::Point p1 = cv::Point();
        cv::Point p2 = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                float32_t sumVal = 0.0F;

                bool withinBounds = true;

                for (uint32_t i = 0; i < 3U && withinBounds; i++)
                {
                    for (uint32_t j = 0; j < 3U && withinBounds; j++)
                    {
                        const uint32_t xIdx = x + j - 1U;
                        const uint32_t yIdx = y + i - 1U;

                        if (xIdx >= 0U && xIdx < imSize.width && yIdx >= 0U && yIdx < imSize.height)
                        {
                            p1.x = j;
                            p1.y = i;

                            p2.x = xIdx;
                            p2.y = yIdx;

                            if (horizontal)
                                sumVal += horizontalFilter.at<float32_t>(p1) * static_cast<float32_t>(img.at<imgType>(p2));
                            else
                                sumVal += verticalFilter.at<float32_t>(p1) * static_cast<float32_t>(img.at<imgType>(p2));
                        }
                        else {
                            withinBounds = false;
                            break;
                        }
                    }
                }

                p2.x = x;
                p2.y = y;

                outImg.at<uint8_t>(p2) = abs(withinBounds ? sumVal : 0);
            }
        }

        return outImg;
    }

    template <typename imgType>
    cv::Mat threshold(const cv::Mat& in, const float32_t threshold, bool keepAbove = true)
    {
        cv::Size imSize = in.size();

        cv::Mat out = cv::Mat(in.size(), CV_8UC1);

        cv::Point p = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                if (keepAbove)
                    out.at<uint8_t>(p) = in.at<imgType>(p) > threshold ? 1U : 0U;
                else
                    out.at<uint8_t>(p) = in.at<imgType>(p) < threshold ? 1U : 0U;

            }
        }

        return out;
    }

    template <typename imgType>
    cv::Mat computeGrad(const cv::Mat& in)
    {
        const cv::Mat horizontal = sobelFilter<imgType>(in, true);
        const cv::Mat vertical = sobelFilter<imgType>(in, false);

        cv::Size imSize = in.size();

        cv::Mat out = cv::Mat(in.size(), CV_8UC1);

        cv::Point p = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                const int h = horizontal.at<uint8_t>(p);
                const int v = vertical.at<uint8_t>(p);

                const int h2 = pow(h, 2);
                const int v2 = pow(v, 2);

                const int sum = h2 + v2;

                const int mag = std::sqrt(sum);

                out.at<uint8_t>(p) = mag;
            }
        }

        return out;
    }

private:
    void resetFilters();
    
    cv::Mat verticalFilter;
    cv::Mat horizontalFilter;
};

#endif // IMAGEFILTER
