// CVMaster.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "random_featuredetector.h"
#include "feature_renderer.h"
#include "imagefilter.h"
#include "sobelfilter.h"
#include "harriscorner_featuredetector.h"
#include "checkerboarddetector.h"

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
    // Test hough line detector

    HoughTransform lineFinder = HoughTransform();

    cv::Mat lineImage = cv::Mat(400, 400, CV_8UC1);
    lineImage = cv::Scalar(0, 0, 0);

    cv::line(lineImage, cv::Point(150, 100), cv::Point(350, 250), cv::Scalar(255, 255, 255));
    cv::line(lineImage, cv::Point(300, 100), cv::Point(100, 250), cv::Scalar(255, 255, 255));
    cv::line(lineImage, cv::Point(100, 100), cv::Point(400, 100), cv::Scalar(255, 255, 255));
    cv::String lineWindowName = "Vision output"; //Name of the window

    namedWindow(lineWindowName); // Create a window

    imshow(lineWindowName, lineImage); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    std::vector<cv::Vec2f> lines = {};

    lineFinder.findLines(lineImage, FeatureList(), lines, 50.0F, 0.5F, 2.0F);

    return 0;
    // ------------------------------------------------------

    // Read the image file
    const Mat image = imread("C:/Dev/Data/Images/Calibration/Tellak/6.png");

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
    resize(greyImage, greyImage, Size(640, 480));//Size(1296, 968));

    cv::String windowName = "Vision output"; //Name of the window

    namedWindow(windowName); // Create a window

    imshow(windowName, image); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    HarrisFeatureDetector featureDetector = HarrisFeatureDetector();
    FeatureRenderer featureRenderer = FeatureRenderer();

    cv::Mat contours = cv::Mat();

    featureDetector.processFeatures(greyImage, contours);

    CheckerboardFinder checkerboardFinder = CheckerboardFinder();

    FeatureList features = FeatureList();

    featureDetector.getFeatures(features);
    featureRenderer.processImage(greyImage, features);

    std::vector<int32_t> lineIdx = {};

    imshow(windowName, greyImage); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    destroyWindow(windowName); //destroy the created window

    checkerboardFinder.detectBoards(greyImage, features, lineIdx);

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