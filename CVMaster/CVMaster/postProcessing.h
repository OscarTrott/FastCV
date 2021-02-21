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

    static cv::Mat removeSingles(const cv::Mat& in)
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

                if (sum > 3)
                {
                    out.at<uint8_t>(p) = 1;
                }
                else
                {
                    out.at<uint8_t>(p) = 0;
                }
            }
        }

        return out;
    }

    static int transitionCount(const int (&cells)[8])
    {
        int gradients = 0;
        int sum = 0;

        for (unsigned char i = 0; i < 8; i++)
        {
            if (cells[i] == 0 && cells[(i + 1) % 8] == 1)
            {
                gradients++;
            }
            sum += cells[i];
        }

        return gradients;
    }

    static bool checkDuplicatePoint(const int (&cells)[8])
    {
        int sum = 0;

        for (unsigned char i = 0; i < 8; i++)
        {
            sum += cells[i];
        }

        for (unsigned char i = 1; i < 8; i += 2)
        {
            // Check for duplicate point
            if (cells[i] == 1 && cells[(i - 1) % 8] == 1 && cells[(i - 2) % 8] == 0 && cells[(i - 3) % 8] == 1 && sum == 3)
            {
                return true;
            }
            else if (cells[i] == 1 && cells[(i + 1) % 8] == 1 && cells[(i + 2) % 8] == 0 && cells[(i + 3) % 8] == 1 && sum == 3)
            {
                return true;
            }
        }

        return false;
    }

    static cv::Mat thin(const cv::Mat& in)
    {
        // https://rosettacode.org/wiki/Zhang-Suen_thinning_algorithm#C.2B.2B

        cv::Size imSize = in.size();

        // Clone the input
        cv::Mat interim = in.clone();
        cv::Mat out = in.clone();

        cv::Point p = cv::Point();

        // Greedy algorithm, value to say to keep computing thinner lines whilst we keep thinning them
        bool changed = true;

        int count = 0;

        while (changed)
        {
            count++;
            std::cout << "Iteration: " << count << "\n";

            changed = false;

            for (int iteration = 0; iteration < 2; iteration++)
            {
                // Iterate over all points in the image
                for (int32_t y = 0; y < imSize.height; y++)
                {
                    for (int32_t x = 0; x < imSize.width; x++)
                    {
                        // Get the current position
                        p.x = x;
                        p.y = y;

                        if (y == 0 || x == 0 || y == imSize.height - 1 || x == imSize.width - 1 || interim.at<uint8_t>(p) == 0)
                        {
                            out.at<uint8_t>(p) = 0;
                            continue;
                        }

                        // Vectorise the 8 surrounding cells
                        int cellValues[8] = {};

                        int idx = 0;
                        int weight = 0; // The center-center is always 1 here

                        const int dxL[8] = {0,1,1,1,0,-1,-1,-1};
                        const int dyL[8] = {-1, -1, 0, 1, 1, 1, 0, -1};

                        for (int32_t idx = 0; idx < 8; idx++)
                        {
                            const int dx = dxL[idx];
                            const int dy = dyL[idx];

                            p.x = x + dx;
                            p.y = y + dy;

                            const uint8_t val = interim.at<uint8_t>(p);

                            cellValues[idx] = val;
                            weight += val;
                        }

                        p.x = x;
                        p.y = y;

                        if (weight >= 2 && weight <= 6 &&
                            (cellValues[0] == 0 || cellValues[2] == 0 || cellValues[4] == 0) &&
                            (cellValues[2] == 0 || cellValues[4] == 0 || cellValues[6] == 0) &&
                            (transitionCount(cellValues) == 1) &&
                            iteration == 0
                            )
                        {
                            out.at<uint8_t>(p) = 0;
                            changed = true;
                        }
                        else if (weight >= 2 && weight <= 6 &&
                            (cellValues[0] == 0 || cellValues[2] == 0 || cellValues[6] == 0) &&
                            (cellValues[0] == 0 || cellValues[4] == 0 || cellValues[6] == 0) &&
                            transitionCount(cellValues) == 1 &&
                            iteration == 1
                            )
                        {
                            out.at<uint8_t>(p) = 0;
                            changed = true;
                        }
                        
                        if (weight >= 2 && weight <= 6 && checkDuplicatePoint(cellValues))
                        {
                            out.at<uint8_t>(p) = 0;
                            interim.at<uint8_t>(p) = 0; // Has to happen otherwise we'll remove all duplicate points instead of just one
                            changed = true;
                        }
                    }
                }

                // Have a copy so we're not changing the scene on each pixel which could affect the next pixel
                interim = out.clone();
            }
        }

        return out;
    }
};
