/**
 *  @brief Class declaration for finding and identifying lines within an image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file hough_linefinder.h
 *
 */

#include "typedefinitions.h"
#include "opencv2/opencv.hpp"
#include "feature.h"
#include "sobelfilter.h"
#include "display.h"

#pragma once

/**
 *	@brief Class to find lines from a contours image (i.e the result of a canny edge detector)
 */
class HoughTransform
{
public:
	/**
	 *	@brief Default constructor.
	 */
	HoughTransform();

	/**
	 *	@brief Default destructor
	 */
	~HoughTransform();


	uint32_t findLines(cv::Mat& image,
		const std::vector<Feature>& features, 
		std::vector<cv::Vec2f>& lines, 
		const float32_t edgeIntensityThreshold = 50.0F, 
		const float32_t thetaResolution = 2.0F, 
		const float32_t phiResolution = 1.0F);

private:

	bool xyToPolar(const uint32_t x, const uint32_t y, const float32_t theta_rad, float32_t& phi);

	bool polarToXY(const float32_t theta_rad, const float32_t phi, float32_t& m, float32_t& y);

	void plotLine(cv::Mat& image, const float32_t theta, const float32_t phi);

	cv::Mat mBins;

	const uint16_t xBinOffset;
	const float32_t pi;
	float32_t thetaResolution_rad;
	Display disp;
};

