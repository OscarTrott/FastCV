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
#include "checkerboard.h"

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

	void clusterLines(std::vector<int8_t>& lineCluster, const std::vector<cv::Vec2f>& line);

	void associateLineFeatures(std::vector<Feature>& features, std::vector<cv::Vec2f>& lines, std::vector<uint16_t>(&featLineAssociation)[1000], std::vector<uint16_t>(&lineFeatAssociation)[1000]);

	void checkerboardFeatureTraversal(std::vector<Checkerboard>& checkerboards);

	static float32_t featureDistance(const Feature f1, const Feature& f2);

	static void findPossibleCheckerboardRowPermutation(std::vector<std::vector<uint16_t>>& checkerboardOutput, const std::vector<float32_t>& distances, const std::vector<uint16_t>& lineFeatures, std::vector<float32_t>& orderedModeDistances, std::vector<uint16_t>& orderedModeDistancesCount);

	static void computeLineFeatureDistances(std::vector<float32_t>& distances, const std::vector<uint16_t>& lineFeatures, const std::vector<Feature>& features);

	static bool addNextLine(Checkerboard& checkerboard, const std::vector<uint16_t>& lineFeatures, const std::vector<Feature> features,
		const std::vector<cv::Vec2f> lines,
		std::vector<int32_t> featureState,
		const std::vector<uint16_t> featLineAssociation[1000],
		const std::vector<uint16_t> lineFeatAssociation[1000],
		cv::Mat& renderingImage);

	static void computeModeFeatureDistances(std::vector<float32_t>& orderedModeDistances, std::vector<uint16_t>& orderedModeDistancesCount, const std::vector<float32_t> distances, const float32_t maxPixelDelta);

	void computeFeatureLineDistance(std::vector<float32_t>& distances, const std::vector<Feature>& features, const cv::Vec2f& lineNormal);

	static void plotLine(cv::Mat& image, const cv::Vec2f& lineNorm);

	static void plotLine(cv::Mat& image, const float32_t theta_rad, const float32_t phi);

	/**
	 *	@param[in] featLineAssociation Indexing by feature i will return the index list of all lines passing through or close to i
	 *	@param[in] lineFeatAssociation Indexing by line j will return the index list of all features along or close to j
	 */
	static void findCheckerboard(const std::vector<Feature> features, const std::vector<cv::Vec2f>, const std::vector<uint16_t> featLineAssociation[1000], const std::vector<uint16_t> lineFeatAssociation[1000], std::vector<Checkerboard> &checkerboards, cv::Mat& renderImg);

	static void plotFeatureList(cv::Mat& image, const FeatureList& features);

	/**
	 *	@brief Finds checkerboard patterns within a given image
	 *
	 *	@param[in] contours The grayscale combination of the xy image intensity derivatives
	 *	@param[in] features The corner features detected within the given image
	 *	@param[out] boardIdx A list the same length as features which defines the index of the board associated with it, -1 if it is not on a board
	 *
	 *	@return An integer representing the number of boards found within the image
	 */
	uint32_t detectBoards(cv::Mat& image, const std::vector<Feature>& features, std::vector<Checkerboard>& boardIdx);

private:
	/**
	 *  @brief An enum defining the values which indicate the states of a particular feature
	 */
	enum FeatureState : int32_t
	{
		unprocessed = -2,
		noBoard = -1,
		processed = 0
	};

	std::vector<Feature> features;
	std::vector<cv::Vec2f> lines;

	// Create line association matrix
	std::vector<uint16_t> featLineAssociation[1000]; // Indexing by feature i will return the index list of all lines passing through or close to i
	std::vector<uint16_t> lineFeatAssociation[1000]; // Indexing by line j will return the index list of all features along or close to j

	std::vector<int32_t> featureState;

	cv::Mat renderingImage;

	static const int32_t maxPixelDelta = 3; // Max inter-feature delta distance to be classified as having the same distance
	const float32_t maxDist = 5.0F; // Maximum distance in pixels of a line from a feature
	static const uint16_t minConsecutiveCheckerboardFeatures = 4U;
	const float32_t minLineAgreement = 0.8F;
	const float32_t maxLineDeviationAngle_deg = 10.0F;
	const float32_t maxLineDeviationAngle_rad = 2.0F * acosf(0.0F) * maxLineDeviationAngle_deg / 180.0F; // Convert degrees to radians

	cv::Size imSize;
};

