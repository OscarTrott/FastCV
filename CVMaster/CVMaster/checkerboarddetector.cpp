/**
 *  @brief Class definition for finding and identifying checkerboards within an image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file checkerboarddetector.cpp
 *
 */

#include "checkerboarddetector.h"

CheckerboardFinder::CheckerboardFinder()
{}

CheckerboardFinder::~CheckerboardFinder()
{}

void CheckerboardFinder::clusterLines(std::vector<int8_t>& lineCluster, const std::vector<cv::Vec2f>& line)
{

}

float32_t dot(Feature f1, Feature f2)
{
    return f1.x * f2.x + f1.y * f2.y;
}

float32_t dot(Feature f1, cv::Vec2f f2)
{
    return f1.x * f2[0] + f1.y * f2[1];
}

float32_t dot(cv::Vec2f f1, cv::Vec2f f2)
{
    return f1[0] * f2[0] + f1[1] * f2[1];
}

void CheckerboardFinder::computeFeatureLineDistance(std::vector<float32_t>& distances, const std::vector<Feature>& features, const cv::Vec2f& lineNormal)
{
    for (uint16_t i = 0U; i < features.size(); i++)
    {
        const float32_t AB = dot(features[i], lineNormal);
        const float32_t AA = dot(lineNormal, lineNormal);

        const float32_t x = lineNormal[0] * AB / AA;
        const float32_t y = lineNormal[1] * AB / AA;

        distances[i] = std::sqrtf(pow(x - lineNormal[0], 2) + pow(y - lineNormal[1], 2));
    }
}

uint32_t CheckerboardFinder::detectBoards(cv::Mat& image, const std::vector<Feature>& features, std::vector<int32_t>& boardIdx)
{
    imSize = image.size();

    const uint32_t cFeatureCount = features.size();

    // Initialise the board indices to -2 indicating that they haven't been processed yet
    boardIdx = std::vector<int32_t>(cFeatureCount, featureState::unprocessed);

    HoughTransform hougher = HoughTransform();

    std::vector<cv::Vec2f> lines = {};
    std::vector<std::vector<uint8_t>> lineFeatures;

    const uint32_t lineCount = hougher.findLines(image, features, lines, 200.0F);

    const int16_t INVALIDCLUSTERINDEX = -1;

    std::vector<int8_t> lineClusterIdx = std::vector<int8_t>(lines.size(), INVALIDCLUSTERINDEX);

    //clusterLines(lineClusterIdx, lines);

    std::vector<float32_t> distances = std::vector<float32_t>(features.size(), -1.0);

    uint16_t nextClusterIdx = 0U;
    const float32_t maxDist = 5.0F; // Maximum distance in pixels

    // Create line association matrix
    std::vector<uint16_t> featLineAssociation[1000] = {}; // Indexing by feature i will return the index list of all lines passing through or close to i
    std::vector<uint16_t> lineFeatAssociation[1000] = {}; // Indexing by line j will return the index list of all features along or close to j

    // Go through all features and associate lines and features
    for (uint16_t i = 0U; i < lineCount; i++)
    {
        const cv::Vec2f& lineTangent = lines[i];

        computeFeatureLineDistance(distances, features, lineTangent);

        for (uint16_t j = 0U; j < features.size(); j++)
        {
            if (distances[j] < maxDist)
            {
                // Associate line and feature
                featLineAssociation[j].push_back(i);
                lineFeatAssociation[i].push_back(j);
            }
        }
    }

    const float32_t maxPixelDelta = 10.0F;
    const uint16_t minConsecutiveCheckerboardFeatures = 4U;

    for (uint16_t featIdx = 0U; featIdx < cFeatureCount; featIdx++)
    {
        // Pick a feature
        const std::vector<uint16_t>& lineIndexList = featLineAssociation[featIdx];

        std::vector<uint16_t> featureIndexes;

        bool isCheckerboard = true;

        for (uint16_t lineIdx = 0U; lineIdx < lineIndexList.size(); lineIdx++)
        {
            const cv::Vec2f& lineNormVec = lines[lineIndexList[lineIdx]];

            const float32_t theta_rad = atan2(lineNormVec[1], lineNormVec[0]);
            const float32_t phi = sqrt(pow(lineNormVec[0], 2) + pow(lineNormVec[1], 2));

            std::vector<uint16_t>& lineFeatures = lineFeatAssociation[lineIdx];

            if (lineFeatures.size() <= 1)
                continue;
            
            std::vector<float32_t> distances;

            // Feature order should already be in ascending y
            for (uint16_t lineFeatIdx = 0U; lineFeatIdx < lineFeatures.size() - 1U; lineFeatIdx++)
            {
                float32_t dist = sqrt(pow(features[lineFeatures[lineFeatIdx]].x - features[lineFeatures[lineFeatIdx + 1]].x, 2) + pow(features[lineFeatures[lineFeatIdx]].y - features[lineFeatures[lineFeatIdx + 1]].y, 2));

                distances.push_back(dist);
            }

            float32_t modeDist = 0.0F;

            std::vector<float32_t> orderedModeDistances;
            std::vector<uint16_t> orderedModeDistancesCount;

            uint16_t modeIdx = 0U;

            for (uint16_t i = 0U; i < distances.size(); i++)
            {
                const float32_t distVal = distances[i];

                for (uint16_t j = 0U; j < orderedModeDistances.size(); j++)
                {
                    if (abs(orderedModeDistances[j] - distances[i]) < maxPixelDelta)
                    {
                        // Update entry
                        orderedModeDistancesCount[j]++;
                        orderedModeDistances[j] += (orderedModeDistances[j] - distances[i]) / orderedModeDistancesCount[j];
                        if (orderedModeDistancesCount[j] > orderedModeDistancesCount[modeIdx])
                        {
                            modeIdx = j;
                        }
                    } 
                    else
                    {
                        // Add entry
                        orderedModeDistances.push_back(distances[i]);
                        orderedModeDistancesCount.push_back(1U);
                    }
                }
            }

            std::vector<uint16_t> orderedModeFeatures;

            for (uint16_t i = 0U; i < orderedModeDistancesCount.size(); i++)
            {
                // A possible line of features along checkerboard
                if (orderedModeDistancesCount[i] > minConsecutiveCheckerboardFeatures)
                {
                    bool last = false;

                    // Find features which fit the spacing criteria
                    for (uint16_t j = 0U; j < distances.size(); j++)
                    {
                        if (abs(distances[i] - orderedModeDistancesCount[modeIdx]) < maxPixelDelta)
                        {
                            orderedModeFeatures.push_back(lineFeatures[i]);
                            last = true;
                        }
                        else if (last)
                        {
                            last = false;
                            orderedModeFeatures.push_back(lineFeatures[i]);
                        }
                    }
                }
            }
        }

        if (isCheckerboard)
        {

        }
    }

    for (uint16_t featIdx = 0U; featIdx < cFeatureCount; featIdx++)
    {
        // Pick a feature
        const std::vector<uint16_t>& lineIndexList = featLineAssociation[featIdx];

        cv::Mat rendering = image.clone();

        float64_t min, max = 0.0;

        cv::minMaxLoc(rendering, &min, &max);

        rendering = 255 * rendering / max;

        cv::cvtColor(rendering, rendering, cv::COLOR_GRAY2BGR);
        rendering.convertTo(rendering, CV_8UC3);

        cv::circle(rendering, cv::Point(features[featIdx].x, features[featIdx].y), 4, cv::Scalar(0, 255, 0));

        // Plot all lines that go through the feature
        for (uint16_t i = 0U; i < lineIndexList.size(); i++)
        {
            const cv::Vec2f& lineNormVec = lines[lineIndexList[i]];
            hougher.plotLine(rendering, atan2(lineNormVec[1], lineNormVec[0]), sqrt(pow(lineNormVec[0], 2) + pow(lineNormVec[1], 2)));
        }

        Display disp;

        disp.showImg(rendering, "lines through feature 10");
    }

    return 2;
}
