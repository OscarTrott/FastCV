#include "opencv2/opencv.hpp"
#include "feature.h"

#pragma once

#ifndef CHECKERBOARD_H
#define CHECKERBOARD_H

/**
 *  @class Checkerboard
 *  @brief Class representing a single checkerboard
 *  @details Contains the feature positions (in image and )
 */
class Checkerboard {
private:
    static const uint16_t MAXSIZE = 100U;

    typedef Feature row[MAXSIZE];

    // Height and width in corners (e.g a checkerboard containing 1 square, would have a height and width of 2)
    uint16_t height;
    uint16_t width;

    Feature features[MAXSIZE][MAXSIZE];

public:
    Checkerboard() : height(0U), width(0U), features()
    {}

    Checkerboard(const uint16_t height_, const uint16_t width_) : height(height_), width(width_), features()
    {
        assert(height < MAXSIZE);
        assert(width < MAXSIZE);
    }

    void setHeight(const uint16_t height_)
    {
        assert(height_ < MAXSIZE);

        height = height_;
    }

    void setWidth(const uint16_t width_)
    {
        assert(width_ < MAXSIZE);

        width = width_;
    }

    row& operator[] (const uint16_t rowIdx)
    {
        assert(rowIdx < MAXSIZE);

        return features[rowIdx];
    }
};

#endif // !CHECKERBOARD_H

