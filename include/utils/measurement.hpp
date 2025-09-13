/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.hpp
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class - ROS2 Version
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Dense>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "core/InEKF.h"

enum MeasurementType
{
  EMPTY,
  IMU,
  JOINT_STATE,
  GPS_VELOCITY,
  CAMERA_ODOM,
  VELOCITY
};

class Measurement
{
  struct MeasurementHeader
  {
    uint64_t seq;
    double stamp;
    std::string frame_id;

    MeasurementHeader()
    {
      seq = 0;
      stamp = 0.0;
      frame_id = "";
    }

    MeasurementHeader(const std_msgs::msg::Header &header_in)
    {
      // ROS2 doesn't have seq field in Header
      seq = 0; // Can implement custom sequence tracking if needed
      // Convert from ROS2 time to double (seconds)
      stamp = header_in.stamp.sec + header_in.stamp.nanosec / 1000000000.0;
      frame_id = header_in.frame_id;
    }
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Measurement();
  virtual ~Measurement() = default;

  MeasurementHeader header;
  void setHeader(const std_msgs::msg::Header &header_in);

  double getTime() const;
  MeasurementType getType();

  friend std::ostream &operator<<(std::ostream &os, const Measurement &m);

protected:
  MeasurementType type_;
};

struct MeasurementCompare
{
  bool operator()(Measurement &lhs, Measurement &rhs) const
  {
    return lhs.getTime() > rhs.getTime();
  }
};

#endif
