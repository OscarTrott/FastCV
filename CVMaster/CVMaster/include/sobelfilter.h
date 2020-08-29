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
class SobelFilter : public IRenderer {

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
    bool sobelFilter(cv::Mat& img, cv::Mat& outImg)
    {
        const cv::Size& imSize = img.size();

        cv::Point p1 = cv::Point();
        cv::Point p2 = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                float32_t sumValX = 0.0F;
                float32_t sumValY = 0.0F;

                bool withinBounds = true;

                for (uint32_t i = 0; i < 3U; i++)
                {
                    for (uint32_t j = 0; j < 3U; j++)
                    {
                        const uint32_t xIdx = x + j - 1U;
                        const uint32_t yIdx = y + i - 1U;

                        if (xIdx >= 0U && xIdx < imSize.width && yIdx >= 0U && yIdx < imSize.height)
                        {
                            p1.x = j;
                            p1.y = i;

                            p2.x = xIdx;
                            p2.y = yIdx;

                            sumValX += 1.0F / 4.0F * horizontalFilter.at<float32_t>(p1) * static_cast<float32_t>(img.at<imgType>(p2));
                            sumValY += 1.0F / 4.0F * verticalFilter.at<float32_t>(p1) * static_cast<float32_t>(img.at<imgType>(p2));
                        }
                        else {
                            withinBounds = false;
                        }
                    }
                }

                p2.x = x;
                p2.y = y;

                outImg.at<cv::Vec2f>(p2)[0] = abs(withinBounds ? sumValX : 0);
                outImg.at<cv::Vec2f>(p2)[1] = abs(withinBounds ? sumValY : 0);
            }
        }


        cv::Mat xyList[2];
        cv::split(outImg, &xyList[0]);

        cv::normalize(xyList[0], xyList[0], 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        cv::normalize(xyList[1], xyList[1], 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

        xyList[0].convertTo(xyList[0], CV_8UC1);
        xyList[1].convertTo(xyList[1], CV_8UC1);

        cv::String windowName = "Vision output"; //Name of the window

        cv::namedWindow(windowName); // Create a window

        cv::imshow(windowName, xyList[0]); // Show our image inside the created window.

        cv::waitKey(0); // Wait for any keystroke in the window

        cv::imshow(windowName, xyList[1]); // Show our image inside the created window.

        cv::waitKey(0); // Wait for any keystroke in the window

        return true;
    }

private:
    
    void resetFilters();
    
    
    cv::Mat verticalFilter;
    cv::Mat horizontalFilter;

};

#endif // IMAGEFILTER
