// CVMaster.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "random_featuredetector.h"
#include "feature_renderer.h"
#include "imagefilter.h"
#include "sobelfilter.h"
#include "harriscorner_featuredetector.h"
#include "checkerboarddetector.h"
#include "localIntensityDimming.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Mat src, src_gray;
int thresh = 100;
int max_thresh = 255;
const char* source_window = "Source image";
const char* corners_window = "Corners detected";

void cornerHarris_demo(int, void*);

int main(int argc, char** argv)
{
    /*const int featRows = 5;
    const int featCols = 5;

    const int imWidth = 500, imHeight = 500;

    const int firstFeatPosX = 150;
    const int firstFeatPosY = 150;

    std::vector<Feature> feats;

    for (int row = 0; row < featRows; row++)
    {
        for (int col = 0; col < featCols; col++)
        {
            feats.push_back(Feature(firstFeatPosX + col * 40, firstFeatPosY + row * 40));
        }
    }

    FeatureRenderer featureRender = FeatureRenderer();

    Mat grey = Mat(imHeight, imWidth, CV_8UC1);
    grey = 0U;
    Mat greyRender = grey;
    featureRender.processImage(greyRender, feats);
    cv::String windowsName = "Vision output"; //Name of the window

    //imshow(windowsName, grey); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    HoughTransform hougher = HoughTransform();

    grey.convertTo(grey, CV_32FC1);

    std::vector<cv::Vec2f> lines;
    hougher.findLines(grey, feats, lines, -0.5F);

    CheckerboardFinder checkerboardsFinder = CheckerboardFinder();
    std::vector<uint16_t> featLineAssociation[1000] = {};
    std::vector<uint16_t> lineFeatAssociation[1000] = {};
    checkerboardsFinder.associateLineFeatures(feats, lines, featLineAssociation, lineFeatAssociation);

    std::vector<Checkerboard> checkerboard;
    checkerboardsFinder.findCheckerboard(feats, lines, featLineAssociation, lineFeatAssociation, checkerboard, greyRender);

    // Finish early
    return 0;*/

    // Read the image file
    const Mat image = imread("C:/Dev/Data/Images/Calibration/Tellak/7.png");

    if (image.empty()) // Check for failure
    {
        cout << "Could not open or find the image" << endl;
        system("pause"); //wait for any key press
        return -1;
    }
    /*resize(image, src, Size(640,480));
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    namedWindow(source_window);
    createTrackbar("Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo);
    imshow(source_window, src);
    cornerHarris_demo(0, 0);
    waitKey();

    return 0;
    */
    Mat greyImage = Mat(image.size(), CV_8UC1);
    cv::cvtColor(image, greyImage, COLOR_RGB2GRAY);
#ifdef NDEBUG
    resize(greyImage, greyImage, Size(1296, 968));//Size(640, 480));//
#else
    resize(greyImage, greyImage, Size(1296, 968));//Size(640, 480));
#endif // NDEBUG

    cv::String windowName = "Vision output"; //Name of the window

    namedWindow(windowName); // Create a window

    imshow(windowName, greyImage); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    // Perform local dimming
    LocalDimming dimmer = LocalDimming();
    // dimmer.processImage(greyImage, 5);

    imshow(windowName, greyImage); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    HarrisFeatureDetector featureDetector = HarrisFeatureDetector();
    FeatureRenderer featureRenderer = FeatureRenderer();

    cv::Mat contours = cv::Mat();

    featureDetector.processFeatures(greyImage, contours);

    CheckerboardFinder checkerboardFinder = CheckerboardFinder();

    FeatureList features = FeatureList();

    featureDetector.getFeatures(features);
    featureRenderer.processImage(greyImage, features);

    std::vector<Checkerboard> checkerboards = {};

    imshow(windowName, greyImage); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    destroyWindow(windowName); //destroy the created window

    checkerboardFinder.detectBoards(contours, features, checkerboards);

    greyImage.convertTo(greyImage, CV_8UC3);
    cv::cvtColor(greyImage, greyImage, COLOR_GRAY2BGR);

    for (uint16_t row = 0U; row < checkerboards[0].getHeight(); row++)
    { 
        for (uint16_t col = 0U; col < checkerboards[0].getWidth(); col++)
        {

            const cv::Scalar colour = cv::Scalar(255, 0, 0);
            const uint32_t radius = 5;

            const Feature& feat = checkerboards[0][row][col];

            cv::circle(greyImage, cv::Point(feat.x, feat.y), radius, colour);
        }
    }

    imshow(windowName, greyImage); // Show our image inside the created window.
    waitKey(0); // Wait for any keystroke in the window

    return 0;
}

void cornerHarris_demo(int, void*)
{
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    Mat dst = Mat::zeros(src.size(), CV_32FC1);
    namedWindow(corners_window);
    cornerHarris(src_gray, dst, blockSize, apertureSize, k);
    Mat dst_norm, dst_norm_scaled;
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    for (int i = 0; i < dst_norm.rows; i++)
    {
        for (int j = 0; j < dst_norm.cols; j++)
        {
            if ((int)dst_norm.at<float>(i, j) > thresh)
            {
                circle(dst_norm_scaled, Point(j, i), 5, Scalar(0), 2, 8, 0);
            }
        }
    }

    imshow(corners_window, dst_norm_scaled);
}