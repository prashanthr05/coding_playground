#include <imu_orientation_estimator/imu_orientation_estimator.h>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_estimator");

    std::string imu_topic{"/ardrone/imu"};
    std::string mag_topic{"/ardrone/mag"};
    std::string odom_topic{"/ardrone/odometry"};

    IMUFilter imu_filt;
    imu_filt.initializeFilter();
    dynamic_reconfigure::Server<imu_orientation_estimator::ParamsConfig> param_server;
    dynamic_reconfigure::Server<imu_orientation_estimator::ParamsConfig>::CallbackType param_f;
    param_f = boost::bind(&IMUFilter::paramsCallback, &imu_filt, _1, _2);

    // call this after initializing filter otherwise will result in leak
    param_server.setCallback(param_f);

    ros::Subscriber imu_subscriber{imu_filt.nh.subscribe(imu_topic, 1, &IMUFilter::imuCallback, &imu_filt)};
    ros::Subscriber mag_subscriber{imu_filt.nh.subscribe(mag_topic, 1, &IMUFilter::magCallback, &imu_filt)};
    ros::Subscriber odom_subscriber{imu_filt.nh.subscribe(odom_topic, 1, &IMUFilter::odomCallback, &imu_filt)};

    ros::Timer update_timer{imu_filt.nh.createTimer(ros::Duration(imu_filt.m_filter_period), &IMUFilter::doFilterStep, &imu_filt)};

    ROS_INFO("rosrun rqt_pose_view rqt_pose_view to visualize results");
    ros::spin();
    ros::shutdown();
    return 0;
}
