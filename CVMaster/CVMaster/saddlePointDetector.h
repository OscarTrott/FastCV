#pragma once

#include <opencv2/opencv.hpp>
#include "checkerboard.h"

namespace SaddlePoint {

    static cv::Mat response(const cv::Mat& in)
    {
        cv::Size imSize = in.size();

        cv::Mat out = cv::Mat(imSize, CV_8UC1);

        cv::Point p = cv::Point();

        for (int32_t y = 1; y < imSize.height - 1; y++)
        {
            for (int32_t x = 1; x < imSize.width - 1; x++)
            {
                p.x = x;
                p.y = y;

                int sum = 0;

                for (int dy = -1; dy < 2; dy++)
                    for (int dx = -1; dx < 2; dx++)
                        sum += in.at<uint8_t>(cv::Point(x + dx, y + dy));

                if (sum > 3)
                    out.at<uint8_t>(p) = 1;
                else
                    out.at<uint8_t>(p) = 0;
            }
        }

        return out;
    }

    static cv::Mat cluster(const cv::Mat& in, const int maxDist = 8)
    {
        cv::Size imSize = in.size();

        cv::Mat out = in.clone();

        cv::Point p = cv::Point();

        for (int32_t y = maxDist; y < imSize.height - (maxDist + 1); y++)
        {
            for (int32_t x = maxDist; x < imSize.width - (maxDist + 1); x++)
            {
                p.x = x;
                p.y = y;

                if (out.at<uint8_t>(p) == 1)
                {
                    cv::Point center{x,y};

                    bool clusterMoved = true;

                    while (clusterMoved)
                    {
                        int points = 0;

                        int xAv = 0, yAv = 0;

                        for (int32_t dy = -maxDist; dy < maxDist + 1; dy++)
                        {
                            for (int32_t dx = -maxDist; dx < maxDist + 1; dx++)
                            {
                                p.x = center.x + dx;
                                p.y = center.y + dy;

                                if (p.x > 0 && p.y > 0 && p.x < imSize.width && p.y < imSize.height && out.at<uint8_t>(p) == 1)
                                {
                                    xAv += p.x;
                                    yAv += p.y;

                                    points++;
                                }
                            }
                        }

                        xAv /= points;
                        yAv /= points;

                        clusterMoved = (xAv != center.x) || (yAv != center.y);

                        if (clusterMoved)
                        {
                            center.x = xAv;
                            center.y = yAv;
                        }
                    }

                    // Set all to zero within the window
                    for (int32_t dy = -maxDist; dy < maxDist + 1; dy++)
                    {
                        for (int32_t dx = -maxDist; dx < maxDist + 1; dx++)
                        {
                            p.x = center.x + dx;
                            p.y = center.y + dy;
                            if (p.x > 0 && p.y > 0 && p.x < imSize.width && p.y < imSize.height)
                                out.at<uint8_t>(p) = 0;
                        }
                    }

                    // Set the center of the cluster as the saddle point
                    out.at<uint8_t>(center) = 1;
                }
            }
        }

        return out;
    }

    static std::vector<cv::Point> extract(const cv::Mat& in)
    {
        cv::Size imSize = in.size();

        std::vector<cv::Point> out;

        cv::Point p = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {

                p.x = x;
                p.y = y;

                if (in.at<uint8_t>(p) == 1)
                    out.push_back(p);
            }
        }

        return out;
    }

    static std::vector<cv::Point> validatePoints(const cv::Mat& centerline, const cv::Mat& saddlePoints)
    {
        cv::Size imSize = centerline.size();

        std::vector<cv::Point> out;

        cv::Point p = cv::Point();

        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                if (centerline.at<uint8_t>(p) == 1 && saddlePoints.at<uint8_t>(p) == 1)
                    out.push_back(p);
            }
        }

        return out;

    }

    enum direction {
        up,
        down,
        left,
        right
    };

    static Checkerboard createCheckerboards(const cv::Mat& centerline, const cv::Mat& saddlePoints)
    {
        cv::Mat visited = centerline.clone();

        cv::Size imSize = centerline.size();
        cv::Mat validSaddlePoints = cv::Mat(imSize, CV_8UC1);
        validSaddlePoints = 0;

        std::queue<cv::Point> nextNodes;
        std::queue<direction> traversedDirection;
        std::queue<cv::Point> prevCheckerboardLocation;

        cv::Point p = cv::Point();

        // Find the first point in the centerline from the top-left
        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                if (centerline.at<uint8_t>(p) == 1 && nextNodes.size() == 0)
                {
                    nextNodes.push(p);
                    traversedDirection.push(direction::down);
                    prevCheckerboardLocation.push(cv::Point(100, 100)); // 100,100 such that we should never go in to a negative position, does this matter?
                }
                if (centerline.at<uint8_t>(p) == 1 && saddlePoints.at<uint8_t>(p) == 1)
                {
                    validSaddlePoints.at<uint8_t>(p) = 1;
                }
            }
        }

        validSaddlePoints = cluster(validSaddlePoints);

        Display::showImg(validSaddlePoints);

        Checkerboard checkerboard = Checkerboard();

        // Traverse the graph
        while (nextNodes.size() != 0)
        {
            const cv::Point currPoint = nextNodes.front();
            const direction prevDirection = traversedDirection.front();
            const cv::Point currCheckerboardLocation = prevCheckerboardLocation.front();
            nextNodes.pop();
            traversedDirection.pop();
            prevCheckerboardLocation.pop();

            if (visited.at<uint8_t>(currPoint) == 1) // Only add the surrounding points if the current node has not been visited yet
            {
                // Add the next points to the queue
                for (int dy = -1; dy < 2; dy++)
                {
                    for (int dx = -1; dx < 2; dx++)
                    {
                        if (dx == 0 && dy == 0)
                            continue;

                        cv::Point next = cv::Point(currPoint.x + dx, currPoint.y + dy);

                        if (!(next.x > 0 && next.y > 0 && next.x < imSize.width && next.y < imSize.height))
                            continue;

                        // Get the next points from the original centerline image as we want to traverse intersections pixels multiple times
                        const int old = centerline.at<uint8_t>(next);

                        if (old == 1)
                        {
                            nextNodes.push(currPoint);
                            
                            // Get the previous direction
                            if (dy < 1 && dx == -1)
                            {
                                traversedDirection.push(direction::left);
                            }
                            else if (dy == 1 && dx < 1)
                            {
                                traversedDirection.push(direction::down);
                            }
                            else if (dy == -1 && dx > -1)
                            {
                                traversedDirection.push(direction::up);
                            }
                            else
                            {
                                traversedDirection.push(direction::right);
                            }

                            currCheckerboardLocation.push();
                        }
                    }
                }

                visited.at<uint8_t>(currPoint) = 0; // Set the current pixel as having been visited
            }
            else
            {
                // We should be at an intersection

                if (validSaddlePoints.at<uint8_t>(currPoint) == 1)
                {

                }
            }
            // Check if the point is on a 
        }

        return checkerboard;
    }

    static std::vector<cv::Point> find(const cv::Mat& in, cv::Mat& responses, cv::Mat& clusteredResponse)
    {
        responses = response(in);

        Display::showImg(responses);

        clusteredResponse = cluster(responses);

        Display::showImg(clusteredResponse);

        return extract(clusteredResponse);
    }

}
