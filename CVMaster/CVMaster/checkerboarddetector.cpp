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

uint32_t CheckerboardFinder::detectBoards(cv::Mat& image, const std::vector<Feature>& features, std::vector<int32_t>& boardIdx)
{
    const uint32_t cFeatureCount = features.size();

    // Initialise the board indices to -2 indicating that they haven't been processed yet
    boardIdx = std::vector<int32_t>(cFeatureCount, featureState::unprocessed);

    HoughTransform hougher = HoughTransform();

    std::vector<cv::Vec2f> lines = {};

    const uint32_t lineCount = hougher.findLines(image, features, lines);

    return 0;
}
