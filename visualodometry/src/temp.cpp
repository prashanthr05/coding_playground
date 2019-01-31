#include <iostream>
#include <memory>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>

class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    std::string m_module_name;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_out_port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_gray_port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_undistorted_port;
    yarp::os::Mutex m_mutex;

    const int m_nr_of_corners_horizontal = 10;
    const int m_nr_of_corners_vertical = 7;

public:
    Processing(const std::string &module_name)
    {
        m_module_name = module_name;
    }

    ~Processing()
    {
    };

    bool open()
    {
        this->useCallback();

        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open( "/" + m_module_name);
        m_out_port.open("/" + m_module_name + "/raw_image:o");
        m_gray_port.open("/" + m_module_name + "/gray_image:o");
        m_undistorted_port.open("/" + m_module_name + "/image:o");
        return true;
    }

    void close()
    {
        m_out_port.close();
        m_gray_port.close();
        m_undistorted_port.close();
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
    }

    void interrupt()
    {
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
    }

    void onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &img )
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &raw_image  = m_out_port.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &gray_image  = m_gray_port.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &out_image  = m_undistorted_port.prepare();

        raw_image.resize(img.width(), img.height());
        out_image.resize(img.width(), img.height());

        cv::Mat in_cv = yarp::cv::toCvMat(raw_image);
        cv::Mat gray_in_cv = yarp::cv::toCvMat(gray_image);
        raw_image = img;

        int nr_of_squares = m_nr_of_corners_horizontal*m_nr_of_corners_vertical;
        cv::Size board_size = cv::Size(m_nr_of_corners_horizontal, m_nr_of_corners_vertical);

        std::vector<std::vector<cv::Point3f> > object_points;
        std::vector<std::vector<cv::Point2f> > image_points;
        std::vector<cv::Point2f> corners;

        int success = 0;

        std::vector<cv::Point3f> vertex_squares;
        for (int i = 0; i < nr_of_squares; i++)
        {
            vertex_squares.push_back(cv::Point3f(i/m_nr_of_corners_horizontal, i%m_nr_of_corners_vertical, 0.0f));
        }

        m_mutex.lock();
        cv::cvtColor(in_cv, gray_in_cv, CV_BGR2GRAY);
        m_mutex.unlock();

        m_mutex.lock();
        bool found = cv::findChessboardCorners(in_cv, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        m_mutex.unlock();

        if (found)
        {
            cv::cornerSubPix(gray_in_cv, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(gray_in_cv, board_size, corners, found);
        }

        imshow("wi", gray_in_cv);

        m_out_port.write();
        m_gray_port.write();
        m_undistorted_port.write();
    }
};

class CalibratorModule : public yarp::os::RFModule
{
    yarp::os::ResourceFinder* m_rf;
    bool m_closing{false};

    std::unique_ptr<Processing> m_processing;
    friend class m_processing;

public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->m_rf = &rf;
        std::string module_name = rf.check("name", yarp::os::Value("grabber"), "module name").asString();
        setName(module_name.c_str());

        m_processing = std::make_unique<Processing>(module_name);
        m_processing->open();
        m_closing = false;
        return true;
    }

    bool close()
    {
        m_processing->interrupt();
        m_processing->close();
        m_processing.reset();
        return true;
    }

    bool quit()
    {
        m_closing = true;
        return true;
    }

    double getPeriod()
    {
        return 0.1;
    }

    bool updateModule()
    {
        return !m_closing;
    }

};


int main(int argc,char *argv[])
{
    yarp::os::Network::init();
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("calibrator: YARP server not available");
        return EXIT_FAILURE;
    }

    CalibratorModule calibModule;
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.configure(argc, argv);

    return calibModule.runModule(rf);
}
