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

                if (sum > 4)
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

    static bool isCornerInArea(cv::Mat& points, cv::Point& startingPoint, cv::Point& vec, const int windowSize = 3)
    {
        cv::Point nextPoint = startingPoint + vec;

        cv::Size imSize = points.size();

        for (int dx = -windowSize; dx < windowSize + 1; dx++)
        {
            for (int dy = -windowSize; dy < windowSize + 1; dy++)
            {
                const cv::Point dPoint = cv::Point(nextPoint.x + dx, nextPoint.y + dy);

                cv::Mat render = points.clone();

                cv::circle(render, startingPoint, 2, cv::Scalar(1));

                cv::circle(render, dPoint, 2, cv::Scalar(1));

                //Display::showImg(render, "Rendering");

                if (dPoint.x < 0 || dPoint.y < 0 || dPoint.x >= imSize.width || dPoint.y >= imSize.height)
                    continue;

                if (points.at<uint8_t>(dPoint) == 1)
                {
                    points.at<uint8_t>(dPoint) = 0; // Clear the point so we don't use it again

                    vec = dPoint - startingPoint;
                    startingPoint = dPoint;

                    return true;
                }
            }
        }

        return false;
    }

    static Checkerboard createCheckerboards(const cv::Mat& centerline, const cv::Mat& saddlePoints)
    {
        cv::Mat visited = centerline.clone();

        cv::Size imSize = centerline.size();

        cv::Mat validSaddlePoints = cv::Mat(imSize, CV_8UC1);
        validSaddlePoints = 0;

        cv::Point p = cv::Point();

        // Find the first point in the centerline from the top-left
        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                if (centerline.at<uint8_t>(p) == 1 && saddlePoints.at<uint8_t>(p) == 1)
                {
                    validSaddlePoints.at<uint8_t>(p) = 1;
                }
            }
        }

        validSaddlePoints = cluster(validSaddlePoints);

        // Find the first point in the centerline from the top-left
        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                if (centerline.at<uint8_t>(p) == 1 && validSaddlePoints.at<uint8_t>(p) == 1)
                {
                    validSaddlePoints.at<uint8_t>(p) = 1;
                }
            }
        }

        Display::showImg(validSaddlePoints);

        Checkerboard checkerboard = Checkerboard();
        
        int currX = 0;
        int currY = 0;

        int pointsAdded = 0;

        cv::Point xVector = {};
        cv::Point yVector = {};

        int maxRowLength = 0;

        // Find the first point and add it to the checkerboard
        for (int32_t y = 0; y < imSize.height; y++)
        {
            for (int32_t x = 0; x < imSize.width; x++)
            {
                p.x = x;
                p.y = y;

                // Only do something at saddle points
                if (validSaddlePoints.at<uint8_t>(p) == 1)
                {
                    if (pointsAdded == 1)
                    {
                        const int dx = x - checkerboard[currY][currX - 1].x;
                        const int dy = y - checkerboard[currY][currX - 1].y;

                        if (abs(dx) > 40 || abs(dy) > 40)
                        {
                            // Outlier
                            currX = 0;
                            pointsAdded = 0;

                        }
                    }

                    // Add the point to the top-left of the checkerboard array
                    if (pointsAdded == 0)
                    {
                        checkerboard[0][0] = Feature(x, y);
                        currX++; // Next point, no matter the direction, is put at the next x position
                        pointsAdded++;
                    }
                    else if (currX == 0) // There are previous rows but this is the first of the next
                    {
                        checkerboard[currY][currX] = Feature(x, y);

                        const int dx = x - checkerboard[currY - 1][currX].x;
                        const int dy = y - checkerboard[currY - 1][currX].y;

                        currX++;
                        pointsAdded++;
                    }
                    else if (currX == 1)
                    {
                        checkerboard[currY][currX] = Feature(x, y);

                        const int dx = x - checkerboard[currY][currX - 1].x;
                        const int dy = y - checkerboard[currY][currX - 1].y;

                        currX++;
                        pointsAdded++;

                        // Form the x vector
                        xVector = cv::Point(dx, dy);

                        // Keep adding points to the current row, until none are found
                        while (isCornerInArea(validSaddlePoints, p, xVector))
                        {
                            checkerboard[currY][currX++] = Feature(p.x, p.y);
                            pointsAdded++;
                        }

                        // There should be at least 3 consecutive points on the checkerboard row to be valid
                        if ((currX < 4 && currY < 2) || (dx > 50 || dy > 50))
                        {
                            checkerboard.setIsValid(false);
                            return checkerboard;
                        }

                        maxRowLength = std::max(maxRowLength, currX);
                        
                        currX = 0;
                        currY++;
                    }
                    else
                    {
                        // Huh, bugger
                    }
                }
            }
        }

        // Set if the checkerboard seems to be valid
        checkerboard.setIsValid(pointsAdded > 20 && maxRowLength > 5 && currY > 5);

        checkerboard.setHeight(currY);
        checkerboard.setWidth(maxRowLength);

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
