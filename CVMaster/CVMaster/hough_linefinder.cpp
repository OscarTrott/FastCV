/**
 *  @brief Class definition for finding and identifying lines within an image
 *
 *  @author Oscar Trott    <oscar.trott@googlemail.com>
 *
 *  @file hough_linefinder.h
 *
 */

#include "hough_linefinder.h"

/**
 *  @details Do nothing
 */
HoughTransform::HoughTransform() : 
	xBinOffset(),
	pi(2.0F * acosf(0.0F)),
	thetaResolution_rad(0.0F)
{
	// Do nothing
}

/**
 *  @details Do nothing
 */
HoughTransform::~HoughTransform()
{
	// Do nothing
}

bool HoughTransform::xyToPolar(const uint32_t x, const uint32_t y, const float32_t theta_rad, float32_t& phi)
{
	// Compute the phi and theta values for this xy position
	// r = x cos(t) + y sin(t)
	const float32_t ct = cos(theta_rad);
	const float32_t st = sin(theta_rad);

	phi = x * ct + y * st;

	return true;
}

void HoughTransform::plotLines(cv::Mat& image, const std::vector<cv::Vec2f>& lines)
{
	for (int i = 0; i < lines.size(); i++)
	{
		const float32_t theta_rad = atan2(lines[i][1], lines[i][0]);
		const float32_t phi = sqrt(pow(lines[i][0], 2) + pow(lines[i][1], 2));

		plotLine(image, theta_rad, phi);
	}
}

void HoughTransform::plotLine(cv::Mat& image, const float32_t theta_rad, const float32_t phi)
{
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
	cv::line(image, p3, p4 , cv::Scalar(0, 255, 0));
}

void HoughTransform::polarToTangent(float32_t& x, float32_t& y , const float32_t theta_rad, const float32_t phi)
{

	const float32_t ct = cosf(theta_rad);
	const float32_t st = sinf(theta_rad);

	x = phi * ct;
	y = phi * st;
}

/**
 *  
 */
uint32_t HoughTransform::findLines(cv::Mat& image, 
	const std::vector<Feature>& features, 
	std::vector<cv::Vec2f>& lines, 
	const float32_t edgeIntensityThreshold, 
	const float32_t thetaResolution, 
	const float32_t phiResolution)
{
	assert(thetaResolution > 0.0F); // Should always be greater than 0, otherwise infinite looping would occurr

	const uint32_t winBorder = 5; // Account for border as gradient intensities are unreliable

	thetaResolution_rad = thetaResolution * pi / 180.0F;

	const cv::Size& imSize = image.size();

	// Compute bin array size
	const float32_t binsPerDeg = std::ceil(1.0F / thetaResolution);
	const float32_t binsPerRad = std::ceil(1.0F / thetaResolution_rad);

	// Maximal phi distance of a line would be at image edge as far from origin (0,0) as possible, which would be diagonal to opposing corner
	const uint32_t maxBinsPhi = static_cast<uint32_t>(std::ceil(phiResolution * std::sqrt(std::pow(imSize.width, 2) + std::pow(imSize.height, 2))));
	const uint32_t binsTheta = static_cast<uint32_t>(std::ceil(binsPerRad * 2.0F * pi)) + 1U;

	// Define bin array
	mBins = cv::Mat(cv::Size(maxBinsPhi * 2 + 1, binsTheta), CV_16UC1);
	mBins = cv::Scalar();

	// Temp point to avoid allocating and deallocating memory constantly
	cv::Point tempPoint(0, 0);

	const int32_t winRadius_pix = 0;

	// Iterate through features
	for (uint32_t featIdx = 0U; featIdx < features.size(); featIdx++)
	{
		// Iterate over window around the feature
		//for (uint32_t dy = winBorder; dy < imSize.height - winBorder; dy++)
		for (int32_t dy = -winRadius_pix; dy < winRadius_pix + 1; dy++)
		{
			for (int32_t dx = -winRadius_pix; dx < winRadius_pix + 1; dx++)
			{
				const int32_t x = features[featIdx].x + dx;
				const int32_t y = features[featIdx].y + dy;

				const float32_t contourVal = image.at<float32_t>(y, x); // For contours
				//const float32_t contourVal = image.at<float32_t>(y, x); // For 8-bit greyscale images

				// Check for borders of image, and that intensity is above threshold
				if (x >= winBorder && y >= winBorder && x < imSize.width - winBorder && imSize.height - winBorder && contourVal > edgeIntensityThreshold)
				{
					// Only iterate over half the angles possible
					for (float32_t currAngle_rad = 0.0F; currAngle_rad < pi; currAngle_rad += thetaResolution_rad)
					{
						assert(currAngle_rad < 2 * pi);

						float32_t phi = 0.0F;
						xyToPolar(x, y, currAngle_rad, phi);

						tempPoint.x = static_cast<uint32_t>(std::roundf(phiResolution * phi + maxBinsPhi));
						tempPoint.y = static_cast<uint32_t>(std::roundf(binsPerRad * currAngle_rad));

 						mBins.at<uint16_t>(tempPoint) += 1;
					}
				}
			}
		}
	}

	// Render the output
	int maxLoc[2] = { 0, 0 };
	float64_t maxIntensity = 0;
	
	cv::minMaxIdx(mBins, nullptr, &maxIntensity, nullptr, &maxLoc[0]);

	const uint32_t threshold = 3;
	//disp.showImg(mBins, "Bin image");

	cv::Mat lineImg = image.clone();
	cv::cvtColor(lineImg, lineImg, cv::COLOR_GRAY2BGR);
	lineImg.convertTo(lineImg, CV_8UC3);

	while (maxIntensity > threshold)
	{
		cv::Vec2f vector = cv::Vec2f(maxLoc[0], maxLoc[1]);

		tempPoint.x = maxLoc[1];
		tempPoint.y = maxLoc[0];

		// Remove from bins
		cv::circle(mBins, tempPoint, 30, 0, -1);

		const float32_t phi = (static_cast<int32_t>(maxLoc[1]) - static_cast<int32_t>(maxBinsPhi)) / phiResolution;
		const float32_t theta_rad = maxLoc[0] / binsPerRad;

		polarToTangent(vector[0], vector[1], theta_rad, phi);
		lines.push_back(vector);

		plotLine(lineImg, theta_rad, phi);

		cv::minMaxIdx(mBins, nullptr, &maxIntensity, nullptr, &maxLoc[0]);
	}

	Display::showImg(lineImg, "line image");

	return lines.size();
}
