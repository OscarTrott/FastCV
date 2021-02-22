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
#include "saddlePointDetector.h"
#include "CCA.h"

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

    float xFac = 1.0F;
    float yFac = 1.0F;

#ifdef NDEBUG
    //xFac = image.size().width / 1296;
    //yFac = image.size().height / 968;
    //resize(greyImage, greyImage, Size(1296, 968));
#else
    xFac = image.size().width / 1296;
    yFac = image.size().height / 968;
    resize(greyImage, greyImage, Size(1296, 968));
#endif // NDEBUG

    cv::Rect roi = cv::Rect(630, 180, 340, 240);
    //cv::rectangle(greyImage, roi, cv::Scalar(255));
    //
    //Display::showImg(greyImage);

    SobelFilter sobel;

    cv::Mat grad = sobel.computeGrad<uint8_t>(greyImage);
    
    const float32_t thresh = 20.0F;

    grad = sobel.threshold<uint8_t>(grad, thresh);

    grad = PostProcessing::removeSingles(grad);

    grad = PostProcessing::fillGaps(grad);
    //grad = grad(roi);
    grad = PostProcessing::thin(grad);

    grad = PostProcessing::removeSingles(grad);

    cv::Mat saddleResponse = cv::Mat(greyImage.size(), CV_8UC1);
    cv::Mat clusteredSaddlePoints = cv::Mat(greyImage.size(), CV_8UC1);

    std::vector<cv::Point> saddlePoints = SaddlePoint::find(grad, saddleResponse, clusteredSaddlePoints);

    cv::Mat featureImage = greyImage.clone();

    FeatureRenderer::processImage(featureImage, saddlePoints);

    Display::showImg(featureImage);
    
    Display::showImg(grad);

    std::vector<cv::Mat> components = cca::findComponents(grad);

    std::vector<Checkerboard> checkerboards;

    for (const cv::Mat& m : components)
    {
        Checkerboard c = SaddlePoint::createCheckerboards(m, clusteredSaddlePoints);

        if (c.isValid())
        {
            checkerboards.push_back(c);

            cv::Mat checkerboardRender = image.clone();
            //resize(checkerboardRender, checkerboardRender, Size(1296, 968));

            for (int i = 0; i < c.getWidth(); i++)
            {
                for (int j = 0; j < c.getHeight(); j++)
                {
                    Feature& f = c[j][i];
                    
                    f.x *= xFac;
                    f.y *= yFac;

                    cv::circle(checkerboardRender, cv::Point(f.x, f.y), 3, cv::Scalar(0, 0, 255));

                    string text = to_string(i) + "," + to_string(j);

                    cv::putText(checkerboardRender, text, cv::Point(f.x, f.y), FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,255,0));
                }
            }

            cv::imshow("Result", checkerboardRender);
            cv::waitKey(0);
        }

        Display::showImg(m);
    }

    // DO THE CALIBRATION, FFS!

    return 0;
}
