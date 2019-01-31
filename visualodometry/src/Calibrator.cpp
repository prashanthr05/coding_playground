#include "Calibrator.h"

ChessBoardCalibrator::ChessBoardCalibrator()
{
    m_intrinsic_calibration_matrix = cv::Mat(3, 3, CV_32FC1);
    // setting focal lengths along X and Y as 1 assuming that camera's aspect ratio is 1
    m_intrinsic_calibration_matrix.ptr<float>(0)[0] = 1;
    m_intrinsic_calibration_matrix.ptr<float>(1)[1] = 1;

    std::cout << "Calibrator: Press [Spacebar] to save the snapshot to the training set... " << std::endl;
}

void ChessBoardCalibrator::setNrOfInternalCornersOnTheBoard(int horizontal, int vertical)
{
    m_nr_of_internal_corners_horizontal = horizontal;
    m_nr_of_internal_corners_vertical = vertical;
    m_board_size = cv::Size(horizontal, vertical);
    m_nr_of_squares = horizontal*vertical;

    for (int j = 0; j < m_nr_of_squares; j++)
    {
        m_projected_corners.push_back(cv::Point3f(j/horizontal, j%horizontal, 0.0f));
    }

    m_is_internal_corner_size_set = true;
}

void ChessBoardCalibrator::setNrOfBoardPoses(int training_set_size)
{
    m_nr_of_boards = training_set_size;
    m_is_board_size_set = true;
}

bool ChessBoardCalibrator::feedRawImage(const cv::Mat& raw_input)
{
    if (!m_is_board_size_set || !m_is_internal_corner_size_set)
    {
        std::cout << "[Error]: Calibrator: Please set training set size and number of internal corners first" << std::endl;
        return false;
    }

    m_raw_image = raw_input;
    if (!m_is_training_set_complete)
    {
        processsChessBoardCorners();
    }
    else
    {
        if (!m_is_camera_calibrated)
        {
            cv::calibrateCamera(m_points_on_the_board_in_3d, m_points_on_the_image_in_2d,
                                m_raw_image.size(), m_intrinsic_calibration_matrix,
                                m_distortion_coefficient_matrix, m_rotation_vectors, m_translation_vectors);
            m_is_camera_calibrated = true;
        }
    }
    return true;
}

bool ChessBoardCalibrator::processsChessBoardCorners()
{
    cv::cvtColor(m_raw_image, m_gray_scale_image, CV_BGR2GRAY);
    bool found = cv::findChessboardCorners(m_raw_image, m_board_size, m_corner_points_in_2d, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found)
    {
        cv::cornerSubPix(m_gray_scale_image, m_corner_points_in_2d, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 0.1));
        cv::drawChessboardCorners(m_gray_scale_image, m_board_size, m_corner_points_in_2d, found);
    }

    cv::imshow("Calibration in process.", m_gray_scale_image);

    int key = cv::waitKey(1);

    if (key == ' ' && found !=0)
    {
        m_points_on_the_image_in_2d.push_back(m_corner_points_in_2d);
        m_points_on_the_board_in_3d.push_back(m_projected_corners);

        m_nr_of_successful_corner_detection++;
        std::cout << "Calibrator: snapshot# " << m_nr_of_successful_corner_detection << " saved" << std::endl;
        if (m_nr_of_successful_corner_detection == m_nr_of_boards)
        {
            m_is_training_set_complete = true;
        }
    }

    return true;
}

bool ChessBoardCalibrator::getProcessedImage(cv::Mat& undistorted_output)
{
    if (!m_is_camera_calibrated)
    {
        return false;
    }

    cv::undistort(m_raw_image, undistorted_output, m_intrinsic_calibration_matrix, m_distortion_coefficient_matrix);
}

void ChessBoardCalibrator::resetFlags()
{
    m_is_training_set_complete = false;
    m_is_camera_calibrated = false;
}

