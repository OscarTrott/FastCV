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
HoughTransform::HoughTransform()
{
}

/**
 *  @details Do nothing
 */
HoughTransform::~HoughTransform()
{
}

/**
 *  
 */
uint32_t HoughTransform::findLines(cv::Mat& image, const std::vector<Feature>& features, std::vector<cv::Vec2f>& lines, const float32_t edgeIntensityThreshold, const float32_t thetaResolution, const float32_t phiResolution)
{
	assert(thetaResolution > 0.0F);

	const uint32_t winBorder = 5; // Pixels

	cv::String windowName = "rendered image"; //Name of the window
	cv::namedWindow(windowName); // Create a window

	cv::Mat rendering = cv::Mat();

	cv::String window2Name = "Vision output"; //Name of the window
	cv::namedWindow(window2Name); // Create a window

	const float32_t pi = 2 * acos(0.0F);
	const float32_t thetaResolution_rad = thetaResolution * pi / 180.0F;

	const cv::Size& imSize = image.size();

	const float32_t binsPerDeg = std::ceil(1.0F / thetaResolution);
	const float32_t binsPerRad = std::ceil(1.0F / thetaResolution_rad);

	const uint32_t maxBinsPhi = std::max(imSize.width, imSize.height);
	const uint32_t binsTheta = static_cast<uint32_t>(std::ceil(binsPerRad * 2 * pi)) + 1U;

	mBins = cv::Mat(cv::Size(maxBinsPhi * 4 + 2, binsTheta), CV_16UC1);

	cv::Point tempPoint(0, 0);
	const cv::Size winSize = cv::Size(9, 9);

	for (uint32_t featIdx = 0U; featIdx < 1; featIdx++)
	{
		for (uint32_t dy = winBorder; dy < imSize.height - winBorder; dy++)
		{
			for (uint32_t dx = winBorder; dx < imSize.width - winBorder; dx++)
			{
				const int32_t x = dx;
				const int32_t y = dy;

				const float32_t contourVal = image.at<uint8_t>(y, x);

				//std::cout << contourVal << std::endl;

				if (x >= 0 && y >= 0 && x < imSize.width && imSize.height && contourVal > edgeIntensityThreshold)
				{
					for (float32_t currAngle_rad = 0.0F; currAngle_rad < 2.0F * pi; currAngle_rad += thetaResolution_rad)
					{
						// Compute the phi and theta values for this xy position
						// r = x cos(t) + y sin(t)
						const float32_t ct = cos(currAngle_rad);
						const float32_t st = sin(currAngle_rad);

						const float32_t phi = x * ct + y * st;

						tempPoint.x = static_cast<uint32_t>(std::roundf(phi + maxBinsPhi * 2.0F));
						tempPoint.y = static_cast<uint32_t>(std::roundf(binsPerRad * currAngle_rad));

 						mBins.at<uint16_t>(tempPoint) += 1;
						/*
						// r = x cos(t) + y sin(t)
						const int32_t xl1 = phi / ct;
						const int32_t xl2 = (phi - imSize.height * st) / ct;
						const int32_t yl1 = 0;
						const int32_t yl2 = imSize.height;

						cv::Mat outputImg;

						image.convertTo(outputImg, CV_8UC4);

						cv::cvtColor(outputImg, outputImg, cv::COLOR_GRAY2BGR);

						cv::line(outputImg, cv::Point(xl1, yl1), cv::Point(xl2, yl2), cv::Scalar(0, 0, 255));
						cv::imshow(window2Name, outputImg);
						// r = x cos(t) + y sin(t)
						         
						mBins.convertTo(rendering, CV_8UC1);
						cv::normalize(rendering, rendering, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat());

						cv::imshow(windowName, rendering); // Show our image inside the created window.

						cv::waitKey(0); // Wait for any keystroke in the window
						
						std::cout << "Current angle: " << 180 * currAngle_rad / pi << std::endl;
						std::cout << "Current phi: " << phi << std::endl;
						std::cout << "Current x: " << tempPoint.x << std::endl;
						std::cout << "Current y: " << tempPoint.y << std::endl;
						*/
					}
				}
			}
		}
	}

	// Render the output

	int maxLoc[2] = { 0, 0 };
	float64_t maxIntensity = 0;
	
	cv::minMaxIdx(mBins, nullptr, &maxIntensity, nullptr, &maxLoc[0]);

	const uint32_t threshold = 10;

	while (maxIntensity > threshold)
	{
		cv::Vec2f vector = cv::Vec2f(maxLoc[0], maxLoc[1]);

		lines.push_back(vector);

		tempPoint.x = maxLoc[1];
		tempPoint.y = maxLoc[0];

		const float32_t phi = maxLoc[1] - maxBinsPhi * 2.0F;

		cv::circle(mBins, tempPoint, 5, 0, -1);

		cv::minMaxIdx(mBins, nullptr, &maxIntensity, nullptr, &maxLoc[0]);

		cv::normalize(mBins, mBins, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

		mBins.convertTo(rendering, CV_8UC1);
		cv::imshow(windowName, rendering); // Show our image inside the created window.
		
		// r = x cos(t) + y sin(t)
		const float32_t theta = maxLoc[0] / binsPerRad;
		const int32_t xl1 = phi / cos(theta);
		const int32_t xl2 = (phi - imSize.height * sin(theta)) / cos(theta);
		const int32_t yl1 = 0;
		const int32_t yl2 = imSize.height;


		std::cout << "Current angle: " << 180 * theta / pi << std::endl;
		std::cout << "Current phi: " << phi << std::endl;
		std::cout << "Current x: " << tempPoint.x << std::endl;
		std::cout << "Current y: " << tempPoint.y << std::endl;

		cv::Mat outputImg;

		image.convertTo(outputImg, CV_8UC4);

		cv::cvtColor(outputImg, outputImg, cv::COLOR_GRAY2BGR);

		cv::line(outputImg, cv::Point(xl1, yl1), cv::Point(xl2, yl2), cv::Scalar(0,0,255));
		cv::imshow(window2Name, outputImg);
		// r = x cos(t) + y sin(t)

		cv::waitKey(0); // Wait for any keystroke in the window
	}


	return lines.size();
}
