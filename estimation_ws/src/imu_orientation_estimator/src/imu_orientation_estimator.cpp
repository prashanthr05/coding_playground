#include <imu_orientation_estimator/imu_orientation_estimator.h>

IMUFilter::IMUFilter()
{
}

void IMUFilter::paramsCallback(imu_orientation_estimator::ParamsConfig& config, uint32_t level)
{
    m_qekf_params.time_step_in_seconds = config.discretization_time_step;
    m_qekf_params.accelerometer_noise_variance = config.accelerometer_noise_variance;
    m_qekf_params.magnetometer_noise_variance = config.magnetometer_noise_variance;
    m_qekf_params.gyroscope_noise_variance = config.gyroscope_noise_variance;
    m_qekf_params.gyro_bias_noise_variance = config.gyro_bias_variance;
    m_qekf_params.initial_orientation_error_variance =  config.initial_orient_variance;
    m_qekf_params.initial_ang_vel_error_variance = config.initial_angvel_variance;
    m_qekf_params.initial_gyro_bias_error_variance = config.initial_gyro_bias_variance;
    m_qekf_params.use_magnetometer_measurements = config.use_magnetometer;

    m_mahony_params.time_step_in_seconds = config.discretization_time_step;
    m_mahony_params.kp = config.kp;
    m_mahony_params.ki = config.ki;
    m_mahony_params.use_magnetometer_measurements = config.use_magnetometer;

    m_qekf->setParameters(m_qekf_params);
    m_mahony->setParameters(m_mahony_params);
}

void IMUFilter::initializeFilter()
{
    m_qekf = std::make_unique<iDynTree::AttitudeQuaternionEKF>();
    m_mahony = std::make_unique<iDynTree::AttitudeMahonyFilter>();

    // TODO: tune them params
    m_qekf_params.time_step_in_seconds = 0.01;
    m_qekf_params.accelerometer_noise_variance = 0.03;
    m_qekf_params.magnetometer_noise_variance = 0.0;
    m_qekf_params.gyroscope_noise_variance = 0.5;
    m_qekf_params.gyro_bias_noise_variance = 10e-4;
    m_qekf_params.initial_orientation_error_variance = 10e-4;
    m_qekf_params.initial_ang_vel_error_variance = 10e-4;
    m_qekf_params.initial_gyro_bias_error_variance = 10e-2;
    m_qekf_params.use_magnetometer_measurements = false;

    m_mahony_params.time_step_in_seconds = 0.01;
    m_mahony_params.kp = 0.3;
    m_mahony_params.ki = 0.07;
    m_mahony_params.use_magnetometer_measurements = false;

    m_qekf->setParameters(m_qekf_params);
    m_qekf->initializeFilter();
    m_mahony->setParameters(m_mahony_params);

    // TODO: tune them initial states
    iDynTree::VectorDynSize x0;
    x0.resize(10);
    x0.zero();
    x0(0) = 1.0;
    iDynTree::Span<double> x0_span(x0.data(), x0.size());
    m_qekf->setInternalState(x0_span);
    m_mahony->setInternalState(x0_span);

    m_ros_publisher = nh.advertise<geometry_msgs::PoseStamped>("/ground_truth/state", 1);
    m_qekf_publisher = nh.advertise<geometry_msgs::PoseStamped>("/qekf/state", 1);
    m_mahony_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mahony/state", 1);
}

void IMUFilter::doFilterStep(const ros::TimerEvent& event)
{
    if (m_imu_msg_recvd && m_odom_msg_recvd)
    {
        if (filt_state == FILTER_STATE::PROPAGATE)
        {
            m_qekf->propagateStates();
            m_mahony->propagateStates();
            filt_state = FILTER_STATE::UPDATE;
        }
        else if (filt_state == FILTER_STATE::UPDATE)
        {
            m_qekf->updateFilterWithMeasurements(m_idyn_acc, m_idyn_gyro, m_idyn_mag);
            m_mahony->updateFilterWithMeasurements(m_idyn_acc, m_idyn_gyro, m_idyn_mag);
            filt_state = FILTER_STATE::PROPAGATE;
        }
        m_qekf->getOrientationEstimateAsQuaternion(m_idyn_qekf_estimate);
        m_mahony->getOrientationEstimateAsQuaternion(m_idyn_mahony_estimate);

        ros::Time now = ros::Time::now();
        m_qekf_attitude.header.frame_id = "/qekf";
        m_qekf_attitude.header.seq++;
        m_qekf_attitude.header.stamp = now;
        m_qekf_attitude.pose.orientation.w = m_idyn_qekf_estimate(0);
        m_qekf_attitude.pose.orientation.x = m_idyn_qekf_estimate(1);
        m_qekf_attitude.pose.orientation.y = m_idyn_qekf_estimate(2);
        m_qekf_attitude.pose.orientation.z = m_idyn_qekf_estimate(3);

        m_mahony_attitude.header.frame_id = "/mahony";
        m_mahony_attitude.header.seq++;
        m_mahony_attitude.header.stamp = now;
        m_mahony_attitude.pose.orientation.w = m_idyn_mahony_estimate(0);
        m_mahony_attitude.pose.orientation.x = m_idyn_mahony_estimate(1);
        m_mahony_attitude.pose.orientation.y = m_idyn_mahony_estimate(2);
        m_mahony_attitude.pose.orientation.z = m_idyn_mahony_estimate(3);

        m_ros_attitude.header.frame_id = "/ground_truth";
        m_ros_attitude.header.seq++;
        m_ros_attitude.header.stamp = now;
        m_ros_attitude.pose.orientation.w = m_ros_estimate(0);
        m_ros_attitude.pose.orientation.x = m_ros_estimate(1);
        m_ros_attitude.pose.orientation.y = m_ros_estimate(2);
        m_ros_attitude.pose.orientation.z = m_ros_estimate(3);

        m_ros_publisher.publish(m_ros_attitude);
        m_qekf_publisher.publish(m_qekf_attitude);
        m_mahony_publisher.publish(m_mahony_attitude);
    }
}

void IMUFilter::imuCallback(const sensor_msgs::ImuPtr& imu)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_idyn_acc(0) = imu->linear_acceleration.x;
    m_idyn_acc(1) = imu->linear_acceleration.y;
    m_idyn_acc(2) = imu->linear_acceleration.z;

    m_idyn_gyro(0) = imu->angular_velocity.x;
    m_idyn_gyro(1) = imu->angular_velocity.y;
    m_idyn_gyro(2) = imu->angular_velocity.z;
    m_imu_msg_recvd = true;
}
void IMUFilter::magCallback(const geometry_msgs::Vector3StampedPtr& mag)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_idyn_mag(0) = mag->vector.x;
    m_idyn_mag(1) = mag->vector.y;
    m_idyn_mag(2) = mag->vector.z;
    m_mag_msg_recvd = true;
}

void IMUFilter::odomCallback(const nav_msgs::OdometryPtr& odom)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ros_estimate(0) = odom->pose.pose.orientation.w; //real part
    m_ros_estimate(1) = odom->pose.pose.orientation.x;
    m_ros_estimate(2) = odom->pose.pose.orientation.y;
    m_ros_estimate(3) = odom->pose.pose.orientation.z;
    m_odom_msg_recvd = true;
}
