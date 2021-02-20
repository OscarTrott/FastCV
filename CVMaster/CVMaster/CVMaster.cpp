// CVMaster.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "random_featuredetector.h"
#include "feature_renderer.h"
#include "imagefilter.h"
#include "sobelfilter.h"
#include "harriscorner_featuredetector.h"
#include "checkerboarddetector.h"
#include "localIntensityDimming.h"
#include "postProcessing.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Mat src, src_gray;
int thresh = 100;
int max_thresh = 255;
const char* source_window = "Source image";
const char* corners_window = "Corners detected";

int main(int argc, char** argv)
{
    // Read the image file
    const Mat image = imread("C:/Dev/Data/Images/Calibration/Tellak/1.png");

    if (image.empty()) // Check for failure
    {
        cout << "Could not open or find the image" << endl;
        system("pause"); //wait for any key press
        return -1;
    }

    Mat greyImage = Mat(image.size(), CV_8UC1);
    cv::cvtColor(image, greyImage, COLOR_RGB2GRAY);
#ifdef NDEBUG
    resize(greyImage, greyImage, Size(1296, 968));//Size(640, 480));//
#else
    resize(greyImage, greyImage, Size(1296, 968));//Size(640, 480));
#endif // NDEBUG

    SobelFilter sobel;

    cv::Mat grad = sobel.computeGrad<uint8_t>(greyImage);

    Display::showImg(grad);

    const float32_t thresh = 30.0F;

    grad = sobel.threshold<uint8_t>(grad, thresh);

    Display::showImg(grad);

    grad = PostProcessing::fillGaps(grad);

    Display::showImg(grad);



    return 0;
}
