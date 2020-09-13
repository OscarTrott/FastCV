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

void CheckerboardFinder::computeFeatureLineDistance(std::vector<float32_t>& distances, const std::vector<Feature>& features, const cv::Vec2f& lineNormal)
{
    for (uint16_t i = 0U; i < features.size(); i++)
    {
        distances[i] = features[i].x * lineNormal[0] + features[i].y * lineNormal[1];
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

    const uint32_t lineCount = hougher.findLines(image, features, lines, 200.0F);

    const int16_t INVALIDCLUSTERINDEX = -1;

    std::vector<int8_t> lineClusterIdx = std::vector<int8_t>(lines.size(), INVALIDCLUSTERINDEX);

    //clusterLines(lineClusterIdx, lines);

    std::vector<float32_t> distances = std::vector<float32_t>(lineCount, -1.0);

    uint16_t nextClusterIdx = 0U;
    const float32_t maxDist = 5.0F;

    // Go through all features
    for (uint16_t i = 0U; i < lineCount; i++)
    {
        
        // This feature has already been associated with a cluster
        if (lineClusterIdx[i] != INVALIDCLUSTERINDEX)
            continue;

        const cv::Vec2f& lineTangent = lines[i];

        computeFeatureLineDistance(distances, features, lineTangent);

        for (uint16_t j = 0U; j < features.size(); j++)
        {
            if (distances[j] < maxDist)
            {

            }
        }
    }

    return 2;
}
