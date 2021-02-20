#pragma once

#include <opencv2/opencv.hpp>

class PostProcessing
{
public:

    static cv::Mat fillGaps(const cv::Mat& in)
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

                if (y == 0 || x == 0 || y == imSize.height - 1 || x == imSize.width - 1)
                {
                    out.at<uint8_t>(p) = 0;
                    continue;
                }

                const uint8_t val = in.at<uint8_t>(p);

                if (val == 1)
                {
                    out.at<uint8_t>(p) = 1;
                }
                else
                {
                    int sum = 0;

                    for (int32_t dx = -1; dx < 2; dx++)
                    {
                        for (int32_t dy = -1; dy < 2; dy++)
                        {
                            p.x = x + dx;
                            p.y = y + dy;

                            if (in.at<uint8_t>(p) == 1)
                                sum++;
                        }
                    }

                    p.x = x;
                    p.y = y;

                    if (sum > 5)
                    {
                        out.at<uint8_t>(p) = 1;
                    }
                    else
                    {
                        out.at<uint8_t>(p) = 0;
                    }
                }
            }
        }

        return out;
    }

};
