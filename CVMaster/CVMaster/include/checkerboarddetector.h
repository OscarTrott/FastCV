/**
 *  @brief Class declaration for finding and identifying checkerboards within an image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file checkerboarddetector.h
 *
 */

#include "feature.h"
#include "hough_linefinder.h"
#include "opencv2/opencv.hpp"

#pragma once

class CheckerboardFinder
{
public:
	/**
	 *	@brief Default constructor
	 */
	CheckerboardFinder();

	/**
	 *	@brief Default destructor
	 */
	~CheckerboardFinder();

	/**
	 *	@brief Finds checkerboard patterns within a given image
	 *
	 *	@param[in] contours The grayscale combination of the xy image intensity derivatives
	 *	@param[in] features The corner features detected within the given image
	 *	@param[out] boardIdx A list the same length as features which defines the index of the board associated with it, -1 if it is not on a board
	 *
	 *	@return An integer representing the number of boards found within the image
	 */
	uint32_t detectBoards(cv::Mat& image, const std::vector<Feature>& features, std::vector<int32_t>& boardIdx);

private:
	/**
	 *  @brief An enum defining the values which indicate the states of a particular feature
	 */
	enum featureState : int32_t
	{
		unprocessed = -2,
		noBoard = -1
	};
};

