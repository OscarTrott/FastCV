#include "display.h"
#include "typedefinitions.h"

Display::Display() : winIdx(0U), defaultStringName("window: ")
{
    // Do nothing
}

void Display::showImg(const cv::Mat& image, const std::string& winName)
{
    std::string winNameOutput = winName;

    if (winNameOutput.size() == 0U)
    {
        winNameOutput = defaultStringName + std::to_string(winIdx++);
    }

    rendering = image.clone();

    float64_t min, max = 0.0;

    cv::minMaxLoc(image, &min, &max);

    rendering = 255 * rendering / max;

    rendering.convertTo(rendering, CV_8UC1);

    cv::namedWindow(winNameOutput); // Create a window
    cv::imshow(winNameOutput, rendering);
    cv::waitKey(0);
}
