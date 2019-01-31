#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <Calibrator.h>

int main()
{
    ChessBoardCalibrator calibrator;
    calibrator.setNrOfBoardPoses(10);
    calibrator.setNrOfInternalCornersOnTheBoard(9, 6);

    cv::VideoCapture capture = cv::VideoCapture(0);

    cv::Mat in;
    capture >> in;

    while (!calibrator.isCalibrated())
    {
        if (!calibrator.feedRawImage(in))
        {
            break;
        }
        capture >> in;
    }

    if (!calibrator.isCalibrated())
    {
        return 0;
    }

    cv::Mat out;
    while(1)
    {
        capture >> in;
        calibrator.feedRawImage(in);
        calibrator.getProcessedImage(out);

        cv::imshow("Raw image", in);
        cv::imshow("Processed image", out);
        cv::waitKey(1);
    }

    return 0;
}
