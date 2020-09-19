/**
 *  @brief Class to hold a single feature data
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file feature.h
 *
*/

#include "typedefinitions.h"
#include <vector>

#ifndef FEATURE
#define FEATURE

/**
 *  @class Feature
 *
 *  @brief Class to hold a single feature
 */
struct Feature {
    
    // Image coordinates, generally will be whole numbers, but sub-pixel accuracy may be possible
    float32_t x;
    float32_t y;

    int64_t descriptor;

    Feature() : x(0U), y(0U), descriptor()
    {
        // Do nothing
    }

    Feature(const uint32_t x_, const uint32_t y_, const int64_t descriptor_ = 0) : x(x_), y(y_), descriptor(descriptor_)
    {
        // Do nothing
    }
};

typedef std::vector<Feature> FeatureList;

#endif
