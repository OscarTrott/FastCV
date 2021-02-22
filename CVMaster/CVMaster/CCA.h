#pragma once

#include "opencv2/opencv.hpp"

namespace cca {
    
    static std::vector<cv::Mat> findComponents(const cv::Mat& in)
    {
        cv::Mat interim = in.clone();

        cv::Size imSize = in.size();

        std::vector<cv::Mat> out;

        cv::Point p = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {

                p.x = x;
                p.y = y;

                if (interim.at<uint8_t>(p) == 1)
                {
                    // Traverse the component

                    cv::Mat newComponent = cv::Mat(imSize, CV_8UC1);
                    newComponent = 0;

                    int size = 1;

                    std::queue<cv::Point> points;

                    points.push(p);
                    interim.at<uint8_t>(p) = 0;

                    while (points.size() != 0)
                    {
                        cv::Point nextPoint = points.front();
                        points.pop();

                        newComponent.at<uint8_t>(nextPoint) = 1;

                        // Add the next points to the queue
                        for (int dy = -1; dy < 2; dy++)
                        {
                            for (int dx = -1; dx < 2; dx++)
                            {
                                if (dx == 0 && dy == 0)
                                    continue;

                                cv::Point currPoint = cv::Point(nextPoint.x + dx, nextPoint.y + dy);

                                if (!(currPoint.x > 0 && currPoint.y > 0 && currPoint.x < imSize.width && currPoint.y < imSize.height))
                                    continue;

                                const int old = interim.at<uint8_t>(currPoint);

                                if (old == 1)
                                {
                                    points.push(currPoint);
                                    interim.at<uint8_t>(currPoint) = 0;
                                    size++;
                                }
                            }
                        }
                    }

                    if (size > 1000)
                    {
                        out.push_back(newComponent);
                    }
                }
            }
        }

        return out;
    }
    
}