#ifndef HUSKYSYSTEM_H
#define HUSKYSYSTEM_H

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp" // ROS2 header
#include <chrono>

#include "estimator/body_estimator.hpp"
#include "system/husky_state.hpp"
#include "utils/husky_data.hpp"
#include "utils/joint_state.hpp"
#include "utils/imu.hpp"
#include "utils/velocity.hpp"

#include "communication/pose_publisher_node.hpp"

// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

// TODO: Singleton design pattern (there should only be one HuskySystem)
class HuskySystem
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Default Constructor - ROS2 uses shared_ptr for nodes
    HuskySystem(rclcpp::Node::SharedPtr node, husky_inekf::husky_data_t *husky_data_buffer);

    ~HuskySystem();

    // Step forward one iteration of the system
    void step();

    void declare_parameters();
    void get_parameters();

private:
    // ROS2 Node pointer
    rclcpp::Node::SharedPtr node_;
    // ROS2 logger
    rclcpp::Logger logger_;
    // ROS2 clock
    rclcpp::Clock::SharedPtr clock_;
    // ROS pose publisher
    PosePublisherNode pose_publisher_node_;
    // ROS timestamp
    rclcpp::Time timestamp_;
    // Husky's current state estimate
    husky_inekf::HuskyState state_;
    // Husky data queues
    husky_inekf::husky_data_t *husky_data_buffer_;
    // Invariant extended Kalman filter for estimating the robot's body state
    husky_inekf::BodyEstimator estimator_;
    // Most recent data packet
    husky_inekf::JointStateMeasurementPtr joint_state_packet_;
    husky_inekf::ImuMeasurementPtr imu_packet_;
    husky_inekf::VelocityMeasurementPtr wheel_velocity_packet_;
    husky_inekf::VelocityMeasurementPtr camera_velocity_packet_;
    husky_inekf::VelocityMeasurementPtr gps_velocity_packet_;

    bool enable_wheel_vel_;
    bool enable_camera_vel_;
    bool enable_gps_vel_;
    Eigen::Matrix<double, 3, 3> wheel_vel_cov_;
    Eigen::Matrix<double, 3, 3> camera_vel_cov_;
    Eigen::Matrix<double, 3, 3> gps_vel_cov_;

    // Update most recent packet to use
    bool updateNextIMU();
    bool updateNextJointState();
    bool updateNextWheelVelocity();
    bool updateNextCameraVelocity();
    bool updateNextGPSVelocity();

    // Publish output path
    void logPoseTxt(const husky_inekf::HuskyState &state_);
    // Output file
    std::string file_name_;
    std::string tum_file_name_;
    std::string vel_est_file_name_; // the velocity estimated by the filter
    std::string bias_est_file_name_;
    std::string vel_input_file_name_; // the velocity used in correction step
    std::string imu_file_name_;

    std::ofstream outfile_;
    std::ofstream tum_outfile_;
    std::ofstream vel_est_outfile_;
    std::ofstream bias_est_outfile_;
    std::ofstream vel_input_outfile_;
    std::ofstream imu_outfile_;

    // Publish path node enable flag
    bool enable_pose_publisher_;
    bool enable_pose_logger_;
    bool enable_debug_logger_;
    bool new_pose_ready_;
    int log_pose_skip_;
    int skip_count_;

    double last_imu_time_;

    // Helper function to get parameter with default value
    template <typename T>
    T get_parameter_or(const std::string &name, const T &default_value);
};

#endif // HUSKYSYSTEM_H