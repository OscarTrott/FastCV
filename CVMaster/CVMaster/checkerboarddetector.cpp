/**
 *  @brief Class definition for finding and identifying checkerboards within an image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file checkerboarddetector.cpp
 *
 */

#include "checkerboarddetector.h"

CheckerboardFinder::CheckerboardFinder() : featLineAssociation(), lineFeatAssociation()
{}

CheckerboardFinder::~CheckerboardFinder()
{}

void CheckerboardFinder::clusterLines(std::vector<int8_t>& lineCluster, const std::vector<cv::Vec2f>& line)
{

}

void CheckerboardFinder::plotLine(cv::Mat& image, const cv::Vec2f& lineNorm)
{
    const float32_t phi = sqrtf(powf(lineNorm[0], 2.0F) + powf(lineNorm[1], 2.0F));
    const float32_t theta_rad = atan2f(lineNorm[1], lineNorm[0]);

    cv::Point p1; // y = 0, clipped by 
    cv::Point p2; // y = height

    const float32_t ct = cosf(theta_rad);
    const float32_t st = sinf(theta_rad);

    if (ct == 0.0F)
    {
        p1.x = 0;
        p1.y = phi;
        p2.x = image.size().width;
        p2.y = phi;
    }
    else
    {
        p1.x = phi / ct;
        p2.x = (phi - image.size().height * st) / ct;
        p1.y = 0;
        p2.y = image.size().height;
    }

    cv::Point p3;
    cv::Point p4;

    p4.x = phi * ct;
    p4.y = phi * st;

    cv::line(image, p1, p2, cv::Scalar(0, 0, 255));
    cv::line(image, p3, p4, cv::Scalar(0, 255, 0));
}

void CheckerboardFinder::plotFeatureList(cv::Mat& image, const FeatureList& features)
{
    image.convertTo(image, CV_8UC3);
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);

    cv::Point tempPoint;
    for (uint16_t i = 0U; i < features.size(); i++)
    {
        tempPoint.x = features[i].x;
        tempPoint.y = features[i].y;
        
        cv::circle(image, tempPoint, 5, cv::Scalar(0, 79, 255));
    }
}

void CheckerboardFinder::associateLineFeatures(std::vector<Feature>& features, std::vector<cv::Vec2f>& lines, std::vector<uint16_t> (&featLineAssociation)[1000], std::vector<uint16_t> (&lineFeatAssociation)[1000])
{
    std::vector<float32_t> distances = std::vector<float32_t>(features.size(), -1.0);

    // Go through all lines and associate lines and features
    for (uint16_t i = 0U; i < lines.size(); i++)
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
}

void CheckerboardFinder::checkerboardFeatureTraversal(std::vector<Checkerboard>& checkerboards)
{
    // Find the checkerboard index of each feature, if it exists on one
    for (uint16_t featIdx = 0U; featIdx < features.size(); featIdx++)
    {
        // Feature has already been processed
        if (featureState[featIdx] != FeatureState::unprocessed)
            continue;

        // Pick a feature
        const std::vector<uint16_t>& lineIndexList = featLineAssociation[featIdx];

        std::vector<uint16_t> featureIndexes;

        bool isCheckerboard = true;

        // Go through every line through the feature and check if it is a checkerboard line
        for (uint16_t lineIdx = 0U; lineIdx < lineIndexList.size(); lineIdx++)
        {
            const cv::Vec2f& lineNormVec = lines[lineIndexList[lineIdx]];

            const float32_t theta_rad = atan2(lineNormVec[1], lineNormVec[0]);
            const float32_t phi = sqrt(pow(lineNormVec[0], 2) + pow(lineNormVec[1], 2));

            FeatureList li;

            li.push_back(features[featIdx]);

            cv::Mat renderCopy = renderingImage.clone();
            plotFeatureList(renderCopy, li);
            plotLine(renderCopy, lineNormVec);

            Display disp;

            disp.showImg(renderCopy);

            std::vector<uint16_t>& lineFeatures = lineFeatAssociation[lineIndexList[lineIdx]];

            if (lineFeatures.size() <= 1)
                continue;

            std::vector<float32_t> distances;

            computeLineFeatureDistances(distances, lineFeatures, features);

            // Compute the inter-feature distance along the line
            std::vector<float32_t> orderedModeDistances;
            std::vector<uint16_t> orderedModeDistancesCount;

            computeModeFeatureDistances(orderedModeDistances, orderedModeDistancesCount, distances, maxPixelDelta);

            // Determine the plausability that a sub-sequence of features are part of a checkerboard
            std::vector<std::vector<uint16_t>> possibleCheckerboards;

            findPossibleCheckerboardRowPermutation(possibleCheckerboards, distances, lineFeatures, orderedModeDistances, orderedModeDistancesCount);

            // We now have a list of possible checkerboard features
            // We need to traverse the graph created by them and add them to a checkerboard
            for (uint16_t checkerboardIdx = 0U; checkerboardIdx < possibleCheckerboards.size(); checkerboardIdx++)
            {
                // Potential checkerboard, to be filled and output if valid set is found
                Checkerboard * potentialCheckerboard;

                // Get feature list reference
                std::vector<uint16_t>& featureIdxList = possibleCheckerboards[checkerboardIdx];

                //Try and identify a checkerboard based on the given line and feature set
                //if (addNextLine(*potentialCheckerboard, featureIdxList))
                //    checkerboards.push_back(*potentialCheckerboard);
                
                //delete potentialCheckerboard;

                // Reset the checkerboard for the next checkerboard to be found
                potentialCheckerboard = new Checkerboard();
            }

        }

        if (featureState[featIdx] == FeatureState::unprocessed)
            featureState[featIdx] = FeatureState::processed;
    }
}

bool isLineCheckerboard(
    const int lineIdx,
    const std::vector<Feature> features,
    const std::vector<cv::Vec2f> lines,
    const std::vector<uint16_t> featLineAssociation[1000],
    const std::vector<uint16_t> lineFeatAssociation[1000],
    std::vector<Checkerboard>& checkerboards,
    std::vector<int32_t>& featureState,
    cv::Mat& renderImg)
{
    return false;
}

void CheckerboardFinder::findCheckerboard(
    const std::vector<Feature> features, 
    const std::vector<cv::Vec2f> lines, 
    const std::vector<uint16_t> featLineAssociation[1000], 
    const std::vector<uint16_t> lineFeatAssociation[1000], 
    std::vector<Checkerboard>& checkerboards,
    cv::Mat& renderImg)
{
    std::vector<int32_t> featureState = std::vector<int32_t>(features.size(), FeatureState::unprocessed);

    // Find the checkerboard index of each feature, if it exists on one
    for (uint16_t featIdx = 0U; featIdx < features.size(); featIdx++)
    {
        // Feature has already been processed
        if (featureState[featIdx] != FeatureState::unprocessed)
            continue;

        // Pick a feature
        const std::vector<uint16_t>& lineIndexList = featLineAssociation[featIdx];

        std::vector<uint16_t> featureIndexes;

        bool isCheckerboard = true;

        // Go through every line through the feature and check if it is a checkerboard line
        for (uint16_t lineIdx = 0U; lineIdx < lineIndexList.size(); lineIdx++)
        {
            const cv::Vec2f& lineNormVec = lines[lineIndexList[lineIdx]];

            const float32_t theta_rad = atan2(lineNormVec[1], lineNormVec[0]);
            const float32_t phi = sqrt(pow(lineNormVec[0], 2) + pow(lineNormVec[1], 2));

            FeatureList li;

            li.push_back(features[featIdx]);

            cv::Mat renderCopy = renderImg.clone();
            plotFeatureList(renderCopy, li);
            plotLine(renderCopy, lineNormVec);

            Display disp;

            disp.showImg(renderCopy);

            // Get all feature indices along the line
            const std::vector<uint16_t>& lineFeatures = lineFeatAssociation[lineIndexList[lineIdx]];

            if (lineFeatures.size() <= 1)
                continue;

            std::vector<float32_t> distances;

            computeLineFeatureDistances(distances, lineFeatures, features);

            // Compute the inter-feature distance along the line
            std::vector<float32_t> orderedModeDistances;
            std::vector<uint16_t> orderedModeDistancesCount;

            computeModeFeatureDistances(orderedModeDistances, orderedModeDistancesCount, distances, maxPixelDelta);

            // Determine the plausability that a sub-sequence of features are part of a checkerboard
            std::vector<std::vector<uint16_t>> possibleCheckerboards;

            findPossibleCheckerboardRowPermutation(possibleCheckerboards, distances, lineFeatures, orderedModeDistances, orderedModeDistancesCount);

            // We now have a list of possible checkerboard features
            // We need to traverse the graph created by them and add them to a checkerboard
            for (uint16_t checkerboardIdx = 0U; checkerboardIdx < possibleCheckerboards.size(); checkerboardIdx++)
            {
                // Potential checkerboard, to be filled and output if valid set is found
                Checkerboard potentialCheckerboard;

                // Get feature list reference
                std::vector<uint16_t>& featureIdxList = possibleCheckerboards[checkerboardIdx];

                //Try and identify a checkerboard based on the given line and feature set
                if (addNextLine(potentialCheckerboard, featureIdxList, features, lines, featureState, featLineAssociation, lineFeatAssociation, renderImg))
                    checkerboards.push_back(potentialCheckerboard);

                // Reset the checkerboard for the next checkerboard to be found
                potentialCheckerboard = Checkerboard();
            }
        }
    }
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

float32_t CheckerboardFinder::featureDistance(const Feature f1, const Feature& f2)
{
    return sqrtf(powf(f1.x  - f2.x, 2.0F) + powf(f1.y - f2.y, 2.0F));
}

void CheckerboardFinder::findPossibleCheckerboardRowPermutation(std::vector<std::vector<uint16_t>>& checkerboardOutput, const std::vector<float32_t>& distances, const std::vector<uint16_t>& lineFeatures, std::vector<float32_t>& orderedModeDistances, std::vector<uint16_t>& orderedModeDistancesCount)
{
    for (uint16_t i = 0U; i < orderedModeDistancesCount.size(); i++)
    {
        // A possible line of features along checkerboard
        if (orderedModeDistancesCount[i] > minConsecutiveCheckerboardFeatures)
        {
            std::vector<uint16_t> tempVec = std::vector<uint16_t>();

            uint16_t sequentialCount = 0U;

            // Find features which fit the spacing criteria
            for (uint16_t j = 0U; j < distances.size(); j++)
            {

                // If the distance is within the delta, add it to a possible checkerboard list and increment count
                if (abs(distances[j] - orderedModeDistances[i]) < maxPixelDelta)
                {
                    tempVec.push_back(lineFeatures[j]);
                    sequentialCount++;
                }
                // If there has been at least one value so far and the current distance is not within the max delta, add the feature to the list and attempt to push it back
                else if (sequentialCount > 0U)
                {
                    tempVec.push_back(lineFeatures[j]);

                    if (sequentialCount > minConsecutiveCheckerboardFeatures)
                        // Add the list to the list of possible vectors
                        checkerboardOutput.push_back(tempVec);

                    // Push back the last value and reset the counter
                    sequentialCount = 0U;

                    // Reset the vector
                    tempVec = std::vector<uint16_t>();
                }
            }

            // Might iterate once too few times
            if (sequentialCount > 0U)
            {
                tempVec.push_back(lineFeatures[lineFeatures.size() - 1]);

                if (sequentialCount > minConsecutiveCheckerboardFeatures)
                    // Add the list to the list of possible vectors
                    checkerboardOutput.push_back(tempVec);

                // Push back the last value and reset the counter
                sequentialCount = 0U;
            }
        }
    }
}

void CheckerboardFinder::computeLineFeatureDistances(std::vector<float32_t>& distances, const std::vector<uint16_t>& lineFeatures, const std::vector<Feature>& features)
{
    // Feature order should already be in ascending y
    for (uint16_t lineFeatIdx = 0U; lineFeatIdx < lineFeatures.size() - 1U; lineFeatIdx++)
    {
        float32_t dist = sqrt(pow(features[lineFeatures[lineFeatIdx]].x - features[lineFeatures[lineFeatIdx + 1]].x, 2) + pow(features[lineFeatures[lineFeatIdx]].y - features[lineFeatures[lineFeatIdx + 1]].y, 2));

        distances.push_back(dist);
    }
}

void CheckerboardFinder::computeModeFeatureDistances(std::vector<float32_t>& orderedModeDistances, std::vector<uint16_t>& orderedModeDistancesCount, const std::vector<float32_t> distances, const float32_t maxPixelDelta)
{
    for (uint16_t i = 0U; i < distances.size(); i++)
    {
        const float32_t distVal = distances[i];

        bool added = false;

        for (uint16_t j = 0U; j < orderedModeDistances.size(); j++)
        {
            if (abs(orderedModeDistances[j] - distances[i]) < maxPixelDelta)
            {
                added = true;

                // Update entry
                orderedModeDistancesCount[j]++;
                orderedModeDistances[j] += (distances[i] - orderedModeDistances[j]) / orderedModeDistancesCount[j];
            }
        }

        // If an existing value was not updated, then add it
        if (!added)
        {
            orderedModeDistances.push_back(distances[i]);
            orderedModeDistancesCount.push_back(1U);

        }
    }

}

bool CheckerboardFinder::addNextLine(Checkerboard& checkerboard, const std::vector<uint16_t>& inLneFeatures, const std::vector<Feature> features,
    const std::vector<cv::Vec2f> lines,
    std::vector<int32_t> featureState,
    const std::vector<uint16_t> featLineAssociation[1000],
    const std::vector<uint16_t> lineFeatAssociation[1000],
    cv::Mat& renderingImage)
{
    const float32_t maxLineDeviationAngle_deg = 10.0F;
    const float32_t maxLineDeviationAngle_rad = 2.0F * acosf(0.0F) * maxLineDeviationAngle_deg / 180.0F; // Convert degrees to radians

    FeatureList li;

    for (uint16_t i = 0U; i < inLneFeatures.size(); i++)
        li.push_back(features[inLneFeatures[i]]);

    cv::Mat renderCopy = renderingImage.clone();
    plotFeatureList(renderCopy, li);

    Display disp;

    disp.showImg(renderCopy);

    // Flag to be set if/when a valid checkerboard has been found so iterations can end (should never find 2 potential solutions)
    bool foundValidSet = false;

    // Iterate over all features in the potential checkerboard row/column
    for (uint16_t checkerboardFeatIdx = 0U; checkerboardFeatIdx < inLneFeatures.size(); checkerboardFeatIdx++)
    {
        // Don't process a feature if it's already been explored
        if (featureState[inLneFeatures[checkerboardFeatIdx]] != FeatureState::unprocessed)
            continue;

        // Get the list of lines which pass through the current feature
        const std::vector<uint16_t>& checkerboardLineList = featLineAssociation[inLneFeatures[checkerboardFeatIdx]];

        // Iterate over all lines going through potential first checkerboard feature
        for (uint16_t checkerboardLineIdx = 0U; checkerboardFeatIdx < checkerboardLineList.size(); checkerboardLineIdx++)
        {
            // Get line norm angle
            const cv::Vec2f& lineNormVec = lines[checkerboardLineList[checkerboardLineIdx]];
            const float32_t theta_rad = atan2f(lineNormVec[1], lineNormVec[0]);

            float32_t dist1 = std::numeric_limits<float32_t>::max(), dist2 = std::numeric_limits<float32_t>::max();

            const std::vector<uint16_t>& lineFeatures = lineFeatAssociation[checkerboardLineList[checkerboardLineIdx]];

            // Don't compute distances if there is only one feature (for line to be made, there should always be a few features along it
            if (lineFeatures.size() > 2)
                for (int16_t testLineFeatIdx = 0U; testLineFeatIdx < lineFeatures.size() - 1; testLineFeatIdx++)
                {
                    const uint16_t testLineFeatureIdx = lineFeatures[testLineFeatIdx];

                    if (testLineFeatureIdx == lineFeatures[checkerboardFeatIdx])
                    {
                        if (testLineFeatIdx == 0)
                        {
                            dist2 = featureDistance(features[testLineFeatureIdx], features[lineFeatures[testLineFeatIdx + 1U]]);
                            dist1 = dist2;
                        }
                        else if (testLineFeatIdx == lineFeatures.size() - 2)
                        {
                            dist2 = featureDistance(features[testLineFeatureIdx], features[lineFeatures[testLineFeatIdx - 1U]]);
                            dist1 = dist2;
                        }
                        else
                        {
                            dist2 = featureDistance(features[testLineFeatureIdx], features[lineFeatures[testLineFeatIdx - 1U]]);
                            dist1 = featureDistance(features[testLineFeatureIdx], features[lineFeatures[testLineFeatIdx + 1U]]);
                        }
                    }
                }

            // Iterate over the other features in the "row" and see if there is an agreeing line
            for (uint16_t checkerboardOtherFeatIdx = 0U; checkerboardOtherFeatIdx < lineFeatures.size(); checkerboardOtherFeatIdx++)
            {
                // Don't use the same point to validate itself
                if (checkerboardOtherFeatIdx == checkerboardFeatIdx)
                    continue;

                // Get the list of lines which pass through the current feature
                const std::vector<uint16_t>& agreementLineList = featLineAssociation[lineFeatures[checkerboardOtherFeatIdx]];

                // Iterate through all lines passing through all other 
                for (uint16_t agreementLineListIdx = 0U; agreementLineListIdx < agreementLineList.size(); agreementLineListIdx++)
                {
                    const float32_t agreementTheta_rad = atan2f(lineNormVec[1], lineNormVec[0]);

                    // Check line iff angles are close
                    if (abs(agreementTheta_rad - theta_rad) > maxLineDeviationAngle_rad)
                        continue;

                    // Get the features along the purported agreement line
                    const std::vector<uint16_t>& agreementLineFeatures = lineFeatAssociation[agreementLineList[agreementLineListIdx]];

                    // Only investigate the line iff there are enough features along it
                    if (agreementLineFeatures.size() > 2)
                    {
                        for (int16_t agreementLineFeatureIdx = 0U; agreementLineFeatureIdx < agreementLineFeatures.size() - 1; agreementLineFeatureIdx++)
                        {
                            const uint16_t agreementLineFeature = agreementLineFeatures[agreementLineFeatureIdx];

                            if (featureState[agreementLineFeature] != FeatureState::unprocessed)
                                continue;

                            // Holds the feature index on the next row to search for a parallel line
                            bool distanceHolds = false;
                            uint16_t nextRowFeatIdx = 0U;

                            // Check if the feature is the one we're considering
                            if (agreementLineFeature == lineFeatures[checkerboardOtherFeatIdx])
                            {
                                if (agreementLineFeatureIdx == 0)
                                {
                                    float32_t distance = featureDistance(features[agreementLineFeature], features[lineFeatures[agreementLineFeatureIdx + 1U]]);

                                    if (abs(distance - dist1) < maxPixelDelta || abs(distance - dist2) < maxPixelDelta)
                                    {
                                        nextRowFeatIdx = lineFeatures[agreementLineFeatureIdx + 1U];
                                        distanceHolds = true;
                                    }

                                }
                                else if (agreementLineFeatureIdx == lineFeatures.size() - 2)
                                {
                                    float32_t distance = featureDistance(features[agreementLineFeature], features[lineFeatures[agreementLineFeatureIdx - 1U]]);

                                    if (abs(distance - dist1) < maxPixelDelta || abs(distance - dist2) < maxPixelDelta)
                                    {
                                        nextRowFeatIdx = lineFeatures[agreementLineFeatureIdx - 1U];
                                        distanceHolds = true;
                                    }
                                }
                                else
                                {
                                    float32_t distance1 = featureDistance(features[agreementLineFeature], features[lineFeatures[agreementLineFeatureIdx + 1U]]);
                                    float32_t distance2 = featureDistance(features[agreementLineFeature], features[lineFeatures[agreementLineFeatureIdx - 1U]]);

                                    if (abs(distance1 - dist1) < maxPixelDelta || abs(distance1 - dist2) < maxPixelDelta)
                                    {
                                        nextRowFeatIdx = lineFeatures[agreementLineFeatureIdx + 1U];
                                    }
                                    else if (abs(distance2 - dist1) < maxPixelDelta || abs(distance2 - dist2) < maxPixelDelta)
                                    {
                                        nextRowFeatIdx = lineFeatures[agreementLineFeatureIdx - 1U];
                                        distanceHolds = true;
                                    }
                                }

                                if (distanceHolds)
                                {
                                    const std::vector<uint16_t>& nextRowLineList = featLineAssociation[agreementLineFeature];

                                    // Iterate over the lines and check that there is one that matches
                                    for (int16_t nextRowLineIdx = 0; nextRowLineIdx < nextRowLineList.size(); nextRowLineIdx++)
                                    {
                                        // Get the row index
                                        uint16_t nextRowLine = nextRowLineList[nextRowLineIdx];

                                        // Get the row normal
                                        const cv::Vec2f& nextRowNorm = lines[nextRowLine];

                                        // Get the norm angle
                                        float32_t nextRowLineAngle = atan2f(nextRowNorm[1], nextRowNorm[0]);

                                        // Check parallelness of the row lines
                                        if (abs(nextRowLineAngle - theta_rad) < maxLineDeviationAngle_rad)
                                        {
                                            checkerboard.setHeight(checkerboard.getHeight() + 1);
                                            checkerboard.setWidth(std::max(static_cast<uint16_t>(lineFeatures.size()), checkerboard.getWidth()));

                                            // Next line is now equidistant and parallel, should be enough checks
                                            for (uint16_t featureToAddIdx = 0U; featureToAddIdx < lineFeatures.size(); featureToAddIdx++)
                                            {
                                                featureState[lineFeatures[featureToAddIdx]] = FeatureState::processed;
                                                checkerboard[checkerboard.getHeight() - 1][featureToAddIdx] = features[lineFeatures[featureToAddIdx]];
                                            }

                                            // Get the features along the next row line and check that they all are as of yet unprocessed
                                            std::vector<uint16_t> newRowFeatures = lineFeatAssociation[nextRowLine];

                                            std::vector<float32_t> distances;

                                            computeLineFeatureDistances(distances, newRowFeatures, features);

                                            // Compute the inter-feature distance along the line
                                            std::vector<float32_t> orderedModeDistances;
                                            std::vector<uint16_t> orderedModeDistancesCount;

                                            computeModeFeatureDistances(orderedModeDistances, orderedModeDistancesCount, distances, maxPixelDelta);

                                            uint16_t newRowFeatIterator = 0U;

                                            while (newRowFeatures[newRowFeatIterator] != nextRowFeatIdx)
                                                newRowFeatIterator++;

                                            bool leftGood = false, rightGood = false;

                                            float32_t modeDist = 0.0F;

                                            for (uint16_t modeDistanceIterator = 0U; modeDistanceIterator < orderedModeDistances.size(); modeDistanceIterator++)
                                            {
                                                // Right checker
                                                if (newRowFeatIterator < distances.size() && abs(distances[newRowFeatIterator] - orderedModeDistances[modeDistanceIterator]) < maxPixelDelta && orderedModeDistancesCount[modeDistanceIterator] > minConsecutiveCheckerboardFeatures)
                                                {
                                                    modeDist = orderedModeDistances[modeDistanceIterator];
                                                    rightGood = true;
                                                }
                                                if (newRowFeatIterator > 0U && abs(distances[newRowFeatIterator - 1] - orderedModeDistances[modeDistanceIterator]) < maxPixelDelta && orderedModeDistancesCount[modeDistanceIterator] > minConsecutiveCheckerboardFeatures)
                                                {
                                                    modeDist = orderedModeDistances[modeDistanceIterator];
                                                    leftGood = true;
                                                }

                                            }

                                            uint16_t leftStart = newRowFeatIterator;
                                            uint16_t rightEnd = newRowFeatIterator;

                                            while (leftGood == true)
                                            {
                                                leftStart--;
                                                leftGood = leftStart > 0U &&
                                                    abs(distances[leftStart - 1] - modeDist) < maxPixelDelta;
                                            }

                                            while (rightGood == true)
                                            {
                                                rightEnd++;
                                                rightGood = rightEnd < distances.size() &&
                                                    abs(distances[rightEnd - 1] - modeDist) < maxPixelDelta;
                                            }

                                            for (uint16_t pushBackPermutation = leftStart; pushBackPermutation <= rightEnd; pushBackPermutation++)
                                            {
                                                newRowFeatures.push_back(newRowFeatures[pushBackPermutation]);
                                            }

                                            addNextLine(checkerboard, newRowFeatures, features, lines, featureState, featLineAssociation, lineFeatAssociation, renderingImage);

                                            // The checkerboard is only valid if there are enough features found on it
                                            return checkerboard.getHeight() >= minConsecutiveCheckerboardFeatures && checkerboard.getWidth() >= minConsecutiveCheckerboardFeatures;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // A checkerboard was never found
    return false;
}

uint32_t CheckerboardFinder::detectBoards(cv::Mat& image, const std::vector<Feature>& inFeatures, std::vector<Checkerboard>& checkerboards)
{
    renderingImage = image.clone();

    renderingImage.convertTo(renderingImage, CV_8UC3);
    cv::cvtColor(renderingImage, renderingImage, cv::COLOR_GRAY2BGR);

    features = inFeatures;

    imSize = image.size();

    const uint32_t cFeatureCount = features.size();

    // Initialise the board indices to -2 indicating that they haven't been processed yet
    featureState = std::vector<int32_t>(cFeatureCount, FeatureState::unprocessed);

    HoughTransform hougher = HoughTransform();

    std::vector<std::vector<uint8_t>> lineFeatures;

    const uint32_t lineCount = hougher.findLines(image, features, lines, 200.0F);

    associateLineFeatures(features, lines, featLineAssociation, lineFeatAssociation);

    checkerboardFeatureTraversal(checkerboards);

    /*
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
    }*/

    return 2;
}
