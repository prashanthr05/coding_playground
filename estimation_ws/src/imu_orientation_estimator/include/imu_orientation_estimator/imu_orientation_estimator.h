# ifndef IMU_ORIENTATION_ESTIMATOR_H
#define IMU_ORIENTATION_ESTIMATOR_H

#include <iostream>
#include <cmath>
#include <memory>
#include <iDynTree/Estimation/AttitudeQuaternionEKF.h>
#include <iDynTree/Estimation/AttitudeMahonyFilter.h>

#include <dynamic_reconfigure/server.h>
#include <imu_orientation_estimator/ParamsConfig.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <mutex>

class IMUFilter
{
public:
    IMUFilter();
    void initializeFilter();
    void doFilterStep(const ros::TimerEvent& event);

    void imuCallback(const sensor_msgs::ImuPtr& imu);
    void magCallback(const geometry_msgs::Vector3StampedPtr& mag);
    void odomCallback(const nav_msgs::OdometryPtr& odom);
    void paramsCallback(imu_orientation_estimator::ParamsConfig& config, uint32_t level);

    ros::NodeHandle nh;
    double m_filter_period{0.01};
private:
    enum FILTER_STATE
    {
        PROPAGATE,
        UPDATE
    };
    FILTER_STATE filt_state{FILTER_STATE::PROPAGATE};
    std::mutex m_mutex;

    bool m_imu_msg_recvd{false};
    bool m_mag_msg_recvd{false};
    bool m_odom_msg_recvd{false};

    iDynTree::LinearAccelerometerMeasurements m_idyn_acc;
    iDynTree::GyroscopeMeasurements m_idyn_gyro;
    iDynTree::MagnetometerMeasurements m_idyn_mag;

    iDynTree::UnitQuaternion m_ros_estimate;
    iDynTree::UnitQuaternion m_idyn_mahony_estimate;
    iDynTree::UnitQuaternion m_idyn_qekf_estimate;

    geometry_msgs::PoseStamped m_ros_attitude;
    geometry_msgs::PoseStamped m_qekf_attitude;
    geometry_msgs::PoseStamped m_mahony_attitude;

    ros::Publisher m_ros_publisher;
    ros::Publisher m_qekf_publisher;
    ros::Publisher m_mahony_publisher;

    std::unique_ptr<iDynTree::AttitudeQuaternionEKF> m_qekf;
    iDynTree::AttitudeQuaternionEKFParameters m_qekf_params;
    std::unique_ptr<iDynTree::AttitudeMahonyFilter> m_mahony;
    iDynTree::AttitudeMahonyFilterParameters m_mahony_params;
};

#endif
