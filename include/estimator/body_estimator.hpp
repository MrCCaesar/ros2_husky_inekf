#pragma once
#ifndef BODYESTIMATOR_H
#define BODYESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "utils/husky_data.hpp"
#include "system/husky_state.hpp"
#include "core/InEKF.h"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include "inekf_msgs/msg/state.hpp"

namespace husky_inekf
{
    class BodyEstimator
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 构造函数，接受ROS2节点指针用于参数和日志
        BodyEstimator(rclcpp::Node::SharedPtr node);
        ~BodyEstimator();

        bool enabled();
        void enableFilter();
        void disable();
        bool biasInitialized();
        void initBias(const ImuMeasurement<double> &imu_packet_in);

        void initState(const ImuMeasurement<double> &imu_packet_in,
                       const JointStateMeasurement &joint_state_packet_in, HuskyState &state);
        void initState(const ImuMeasurement<double> &imu_packet_in,
                       const VelocityMeasurement &velocity_packet_in, HuskyState &state);
        void propagateIMU(const ImuMeasurement<double> &imu_packet_in, HuskyState &state);
        void correctVelocity(const JointStateMeasurement &joint_state_packet_in, HuskyState &state, const Eigen::Matrix<double, 3, 3> &velocity_cov);
        void correctVelocity(const VelocityMeasurement &velocity_packet_in, HuskyState &state, const Eigen::Matrix<double, 3, 3> &velocity_cov);

        inekf::InEKF getFilter() const;
        inekf::RobotState getState() const;

    private:
        // ROS2节点指针，用于参数访问和日志记录
        rclcpp::Node::SharedPtr node_;

        // 声明参数的方法
        void declare_parameters();
        void get_parameters();

        // ROS相关
        std::vector<geometry_msgs::msg::PoseStamped> poses_;

        // InEKF相关
        inekf::InEKF filter_;
        bool enabled_ = false;
        bool bias_initialized_ = false;
        bool static_bias_initialization_ = false;
        bool estimator_debug_enabled_ = false;
        bool use_imu_ori_est_init_bias_ = false;
        std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bias_init_vec_;
        Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba0_ = Eigen::Vector3d::Zero();

        double t_prev_;
        uint32_t seq_;
        double velocity_t_thres_;
        Eigen::Matrix<double, 6, 1> imu_prev_;

        // 参数变量
        double gyroscope_std_;
        double accelerometer_std_;
        double gyroscope_bias_std_;
        double accelerometer_bias_std_;
        std::vector<double> gyroscope_bias_prior_;
        std::vector<double> accelerometer_bias_prior_;
    };

} // end husky_inekf namespace
#endif // BODYESTIMATOR_H#pragma once
#ifndef BODYESTIMATOR_H
#define BODYESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "utils/husky_data.hpp"
#include "system/husky_state.hpp"
#include "core/InEKF.h"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include "inekf_msgs/msg/state.hpp"

namespace husky_inekf
{
    class BodyEstimator
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 构造函数，接受ROS2节点指针用于参数和日志
        BodyEstimator(rclcpp::Node::SharedPtr node);
        ~BodyEstimator();

        bool enabled();
        void enableFilter();
        void disable();
        bool biasInitialized();
        void initBias(const ImuMeasurement<double> &imu_packet_in);

        void initState(const ImuMeasurement<double> &imu_packet_in,
                       const JointStateMeasurement &joint_state_packet_in, HuskyState &state);
        void initState(const ImuMeasurement<double> &imu_packet_in,
                       const VelocityMeasurement &velocity_packet_in, HuskyState &state);
        void propagateIMU(const ImuMeasurement<double> &imu_packet_in, HuskyState &state);
        void correctVelocity(const JointStateMeasurement &joint_state_packet_in, HuskyState &state, const Eigen::Matrix<double, 3, 3> &velocity_cov);
        void correctVelocity(const VelocityMeasurement &velocity_packet_in, HuskyState &state, const Eigen::Matrix<double, 3, 3> &velocity_cov);

        inekf::InEKF getFilter() const;
        inekf::RobotState getState() const;

    private:
        // ROS2节点指针，用于参数访问和日志记录
        rclcpp::Node::SharedPtr node_;

        // 声明参数的方法
        void declare_parameters();
        void get_parameters();

        // ROS相关
        std::vector<geometry_msgs::msg::PoseStamped> poses_;

        // InEKF相关
        inekf::InEKF filter_;
        bool enabled_ = false;
        bool bias_initialized_ = false;
        bool static_bias_initialization_ = false;
        bool estimator_debug_enabled_ = false;
        bool use_imu_ori_est_init_bias_ = false;
        std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bias_init_vec_;
        Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba0_ = Eigen::Vector3d::Zero();

        double t_prev_;
        uint32_t seq_;
        double velocity_t_thres_;
        Eigen::Matrix<double, 6, 1> imu_prev_;

        // 参数变量
        double gyroscope_std_;
        double accelerometer_std_;
        double gyroscope_bias_std_;
        double accelerometer_bias_std_;
        std::vector<double> gyroscope_bias_prior_;
        std::vector<double> accelerometer_bias_prior_;
    };

} // end husky_inekf namespace
#endif // BODYESTIMATOR_H