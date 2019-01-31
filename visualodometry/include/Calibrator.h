#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <iostream>
#include <vector>
#include <memory>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class ChessBoardCalibrator
{
    int m_nr_of_boards{0};
    int m_nr_of_internal_corners_horizontal{0};
    int m_nr_of_internal_corners_vertical{0};
    int m_nr_of_successful_corner_detection{0};
    int m_nr_of_squares{0};

    bool m_is_internal_corner_size_set{false};
    bool m_is_board_size_set{false};
    bool m_is_training_set_complete{false};
    bool m_is_camera_calibrated{false};

    cv::Size m_board_size;

    std::vector<std::vector<cv::Point3f> > m_points_on_the_board_in_3d;
    std::vector<std::vector<cv::Point2f> > m_points_on_the_image_in_2d;

    std::vector<cv::Point3f> m_projected_corners;

    std::vector<cv::Point2f> m_corner_points_in_2d;

    cv::Mat m_raw_image;
    cv::Mat m_gray_scale_image;
    cv::Mat m_processed_image;

    cv::Mat m_intrinsic_calibration_matrix;
    cv::Mat m_distortion_coefficient_matrix;
    std::vector<cv::Mat> m_rotation_vectors;
    std::vector<cv::Mat> m_translation_vectors;

    bool processsChessBoardCorners();

public:
    ChessBoardCalibrator();
    void setNrOfInternalCornersOnTheBoard(int horizontal, int vertical);
    void setNrOfBoardPoses(int training_set_size);
    bool isCalibrated() {return m_is_camera_calibrated; }
    void resetFlags();
    bool feedRawImage(const cv::Mat& raw_input);
    bool getProcessedImage(cv::Mat& undistorted_output);

};

#endif
