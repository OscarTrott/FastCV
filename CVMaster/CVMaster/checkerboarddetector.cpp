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

float32_t CheckerboardFinder::featureDistance(const Feature f1, const Feature& f2)
{
    return sqrtf(powf(f1.x  - f2.x, 2.0F) + powf(f1.y - f2.y, 2.0F));
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

}


uint32_t CheckerboardFinder::detectBoards(cv::Mat& image, const std::vector<Feature>& features, std::vector<Checkerboard>& boardIdx)
{
    imSize = image.size();

    const uint32_t cFeatureCount = features.size();

    // Initialise the board indices to -2 indicating that they haven't been processed yet
    std::vector<int32_t> featureState = std::vector<int32_t>(cFeatureCount, featureState::unprocessed);

    HoughTransform hougher = HoughTransform();

    std::vector<cv::Vec2f> lines = {};
    std::vector<std::vector<uint8_t>> lineFeatures;

    const uint32_t lineCount = hougher.findLines(image, features, lines, 200.0F);

    std::vector<float32_t> distances = std::vector<float32_t>(features.size(), -1.0);

    uint16_t nextClusterIdx = 0U;
    const float32_t maxDist = 5.0F; // Maximum distance in pixels

    // Create line association matrix
    std::vector<uint16_t> featLineAssociation[1000] = {}; // Indexing by feature i will return the index list of all lines passing through or close to i
    std::vector<uint16_t> lineFeatAssociation[1000] = {}; // Indexing by line j will return the index list of all features along or close to j

    // Go through all lines and associate lines and features
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
    const float32_t minLineAgreement = 0.8F;
    const float32_t maxLineDeviationAngle_deg = 10.0F;
    const float32_t maxLineDeviationAngle_rad = 2.0F * acosf(0.0F) * maxLineDeviationAngle_deg / 180.0F; // Convert degrees to radians

    // Find the checkerboard index of each feature, if it exists on one
    for (uint16_t featIdx = 0U; featIdx < cFeatureCount; featIdx++)
    {
        // Feature has already been processed
        if (featureState[featIdx] != featureState::unprocessed)
            continue;

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

            computeLineFeatureDistances(distances, lineFeatures, features);

            // Compute the inter-feature distance along the line
            std::vector<float32_t> orderedModeDistances;
            std::vector<uint16_t> orderedModeDistancesCount;

            computeModeFeatureDistances(orderedModeDistances, orderedModeDistancesCount, distances, maxPixelDelta);

            // Determine the plausability of a sequence of features are part of a checkerboard
            std::vector<std::vector<uint16_t>> possibleCheckerboards;

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
                        if (abs(distances[i] - orderedModeDistances[i]) < maxPixelDelta)
                        {
                            tempVec.push_back(lineFeatures[i]);
                            sequentialCount++;
                        }
                        // If there has been at least one value so far and the current distance is not within the max delta, add the feature to the list and attempt to push it back
                        else if (sequentialCount > 0U)
                        {
                            // Push back the last value and reset the counter
                            sequentialCount = 0U;
                            tempVec.push_back(lineFeatures[i]);

                            if (sequentialCount > minConsecutiveCheckerboardFeatures)
                                // Add the list to the list of possible vectors
                                possibleCheckerboards.push_back(tempVec);

                            // Reset the vector
                            tempVec = std::vector<uint16_t>();
                        }
                    }

                    // Might iterate once too few times
                    if (sequentialCount > 0U)
                    {
                        // Push back the last value and reset the counter
                        sequentialCount = 0U;
                        tempVec.push_back(lineFeatures[i]);

                        if (sequentialCount > minConsecutiveCheckerboardFeatures)
                            // Add the list to the list of possible vectors
                            possibleCheckerboards.push_back(tempVec);
                    }
                }
            }

            // We now have a list of possible checkerboard features
            // We need to traverse the graph created by them and add them to a checkerboard
            for (uint16_t checkerboardIdx = 0U; checkerboardIdx < possibleCheckerboards.size(); checkerboardIdx++)
            {
                // Potential checkerboard, to be filled and output if valid set is found
                Checkerboard potentialCheckerboard;

                // Get feature list reference
                std::vector<uint16_t>& featureIdxList = possibleCheckerboards[checkerboardIdx];

                // Flag to be set if/when a valid checkerboard has been found so iterations can end (should never find 2 potential solutions)
                bool foundValidSet = false;

                // Iterate over all features in the potential checkerboard row/column
                for (uint16_t checkerboardFeatIdx = 0U; checkerboardFeatIdx < featureIdxList.size(); checkerboardFeatIdx++)
                {

                    // Get the list of lines which pass through the current feature
                    const std::vector<uint16_t>& checkerboardLineList = featLineAssociation[featureIdxList[checkerboardFeatIdx]];

                    // Iterate over all lines going through potential first checkerboard feature
                    for (uint16_t checkerboardLineIdx = 0U; checkerboardFeatIdx < checkerboardLineList.size(); checkerboardLineIdx++)
                    {
                        // Get line angle
                        const cv::Vec2f& lineNormVec = lines[checkerboardLineList[checkerboardLineIdx]];
                        const float32_t theta_rad = atan2f(lineNormVec[1], lineNormVec[0]);

                        float32_t agreementCount = 0.0F;

                        float32_t dist1 = 0.0F, dist2 = 0.0F;

                        const std::vector<uint16_t>& lineFeatures = lineFeatAssociation[checkerboardLineList[checkerboardLineIdx]];

                        // Don't compute distances if there is only one feature (for line to be made, there should always be a few features along it
                        if (lineFeatures.size() > 2)
                            for (int16_t testLineFeatIdx = 0U; testLineFeatIdx < lineFeatures.size() - 1; testLineFeatIdx++)
                            {
                                const uint16_t testLineFeatureIdx = lineFeatures[testLineFeatIdx];

                                if (testLineFeatureIdx == featureIdxList[checkerboardFeatIdx])
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
                        for (uint16_t checkerboardOtherFeatIdx = 0U; checkerboardOtherFeatIdx < featureIdxList.size(); checkerboardOtherFeatIdx++)
                        {

                        }

                        if (agreementCount/ static_cast<float32_t>(featureIdxList.size()) > minLineAgreement)
                        {

                            // We've found a checkerboard!!!!
                            foundValidSet = true;
                            break;
                        }

                        if (foundValidSet)
                            break;
                    }

                    if (foundValidSet)
                        break;
                }

                if (foundValidSet)
                {
                    // TODO: Fill out the checkerboard!
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
