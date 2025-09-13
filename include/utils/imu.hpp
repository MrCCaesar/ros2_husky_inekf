#pragma once
#include "utils/measurement.hpp"
#include "sensor_msgs/msg/imu.hpp" // ROS2 message header
#include <stdint.h>
#include <string>
#include <memory>

namespace husky_inekf
{

    template <typename T>
    struct ImuOrientation
    {
        T w, x, y, z;
    };

    template <typename T>
    struct ImuAngularVelocity
    {
        T x, y, z;
    };

    template <typename T>
    struct ImuLinearAcceleration
    {
        T x, y, z;
    };

    template <typename T>
    class ImuMeasurement : public Measurement
    {
    public:
        ImuOrientation<T> orientation;
        ImuAngularVelocity<T> angular_velocity;
        ImuLinearAcceleration<T> linear_acceleration;

        Eigen::Matrix3d getRotation() { return R_; }

        void setRotation()
        {
            Eigen::Quaternion<double> q(orientation.w, orientation.x, orientation.y, orientation.z);
            R_ = q.toRotationMatrix();
        }

        // Construct IMU measurement
        ImuMeasurement()
        {
            type_ = IMU;
        }

        // Overloaded constructor for construction using ROS2 IMU topic
        ImuMeasurement(const sensor_msgs::msg::Imu &imu_msg, std::vector<double> &rotation_imu_body)
        {

            // default is (0, 0.7071, -0.7071, 0)
            Eigen::Quaternion<double> rotation_imu2body(
                rotation_imu_body[0],
                rotation_imu_body[1],
                rotation_imu_body[2],
                rotation_imu_body[3]);
            Eigen::Matrix3d rotation_imu_body_matrix;
            rotation_imu_body_matrix = rotation_imu2body.toRotationMatrix();

            Eigen::Vector3d angular_velocity_imu;
            angular_velocity_imu << imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z;

            Eigen::Vector3d linear_acceleration_imu;
            linear_acceleration_imu << imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z;

            // Transform angular velocity to body frame
            angular_velocity = {
                (rotation_imu_body_matrix * angular_velocity_imu)[0],
                (rotation_imu_body_matrix * angular_velocity_imu)[1],
                (rotation_imu_body_matrix * angular_velocity_imu)[2]};

            // Transform linear acceleration to body frame
            linear_acceleration = {
                (rotation_imu_body_matrix * linear_acceleration_imu)[0],
                (rotation_imu_body_matrix * linear_acceleration_imu)[1],
                (rotation_imu_body_matrix * linear_acceleration_imu)[2]};

            // Map original orientation estimation from IMU to body
            Eigen::Quaternion<double> q(
                imu_msg.orientation.w,
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z);

            Eigen::Quaternion<double> q_body = q * rotation_imu2body;

            orientation = {
                q_body.w(),
                q_body.x(),
                q_body.y(),
                q_body.z()};

            setRotation();
            setHeader(imu_msg.header);
            type_ = IMU;
        }

    private:
        Eigen::Matrix3d R_;
    };

    typedef std::shared_ptr<ImuMeasurement<double>> ImuMeasurementPtr;

} // end husky_inekf namespace