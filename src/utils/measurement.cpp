/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Measurement class - ROS2 Version
 *  @date   September 27, 2018
 **/

#include "utils/measurement.hpp"

using namespace std;
using namespace inekf;

// Construct Empty Measurement
Measurement::Measurement()
{
    header.stamp = 0;
    header.seq = 0;
    header.frame_id = "";
    type_ = EMPTY;
}

void Measurement::setHeader(const std_msgs::msg::Header &header_in)
{
    // ROS2 doesn't have seq field, so we can track it separately if needed
    static uint64_t sequence_counter = 0;
    header.seq = sequence_counter++;
    // Convert ROS2 time to double (seconds)
    header.stamp = header_in.stamp.sec + header_in.stamp.nanosec / 1000000000.0;
    header.frame_id = header_in.frame_id;
}

// Getters
double Measurement::getTime() const
{
    return header.stamp;
}

MeasurementType Measurement::getType()
{
    return type_;
}

// Print measurement
ostream &operator<<(ostream &os, const Measurement &m)
{
    string type_str;
    switch (m.type_)
    {
    case IMU:
        type_str = "IMU";
        break;
    case JOINT_STATE:
        type_str = "JOINT_STATE";
        break;
    case GPS_VELOCITY:
        type_str = "GPS_VELOCITY";
        break;
    case CAMERA_ODOM:
        type_str = "CAMERA_ODOM";
        break;
    case VELOCITY:
        type_str = "VELOCITY";
        break;
    case EMPTY:
        type_str = "EMPTY";
        break;
    default:
        type_str = "Unknown";
    }
    os << "Measurement type: " << type_str << endl;
    return os;
}