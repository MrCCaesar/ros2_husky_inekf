/* ROS2 interfaces */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

/* Husky Data Structures For Internal Program */
#include "utils/husky_data.hpp"
#include "utils/imu.hpp"
#include "utils/joint_state.hpp"
#include "utils/velocity.hpp"
#include "utils/camera_odom.hpp"

#include <thread>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <fstream>

class HuskyComms
{
public:
    HuskyComms(std::shared_ptr<rclcpp::Node> nh, husky_inekf::husky_data_t *husky_data_buffer);
    ~HuskyComms();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg);
    void jointStateVelocityCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg);
    void GPSvelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg);
    void CameraOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr camera_odom_msg);

    void declare_parameters();
    void get_parameters();
    void create_subscribers();

    // ROS2 subscribers
    std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cam_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_record_sub_;

    // Parameters
    std::string imu_topic_;
    std::string joint_topic_;
    std::string wheel_vel_topic_;
    std::string camera_vel_topic_;
    std::string gps_vel_topic_;

    bool enable_wheel_vel_;
    bool enable_camera_vel_;
    bool enable_gps_vel_;

    std::ofstream outfile_;

    double wheel_radius_;
    double vehicle_track_width_;
    double vehicle_length_;

    std::vector<double> rotation_imu_body_;
    Eigen::Matrix4d cam_to_body_;

    husky_inekf::husky_data_t *husky_data_buffer_;
};