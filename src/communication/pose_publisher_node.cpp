#include "communication/pose_publisher_node.hpp"

PosePublisherNode::PosePublisherNode() : Node("pose_publisher_node")
{
    // 声明并获取参数
    declare_parameters();
    get_parameters();

    first_pose_ = {0.0f, 0.0f, 0.0f};

    // 创建发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, 1000);

    RCLCPP_INFO(this->get_logger(), "PosePublisherNode initialized");
    RCLCPP_INFO(this->get_logger(), "Publishing poses on topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose frame: %s", pose_frame_.c_str());
}

PosePublisherNode::~PosePublisherNode()
{
    RCLCPP_INFO(this->get_logger(), "PosePublisherNode shutting down");
}

void PosePublisherNode::declare_parameters()
{
    this->declare_parameter("pose_topic", "/husky/inekf_estimation/pose");
    this->declare_parameter("map_frame_id", "/odom");
    this->declare_parameter("publish_rate", 1000.0);
    this->declare_parameter("pose_skip", 0);
}

void PosePublisherNode::get_parameters()
{
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    pose_frame_ = this->get_parameter("map_frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    pose_skip_ = this->get_parameter("pose_skip").as_int();
}

void PosePublisherNode::posePublish(const husky_inekf::HuskyState &state_)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = pose_frame_;

    pose_msg.pose.pose.position.x = state_.x() - first_pose_[0];
    pose_msg.pose.pose.position.y = state_.y() - first_pose_[1];
    pose_msg.pose.pose.position.z = state_.z() - first_pose_[2];

    pose_msg.pose.pose.orientation.w = state_.getQuaternion().w();
    pose_msg.pose.pose.orientation.x = state_.getQuaternion().x();
    pose_msg.pose.pose.orientation.y = state_.getQuaternion().y();
    pose_msg.pose.pose.orientation.z = state_.getQuaternion().z();

    RCLCPP_DEBUG(this->get_logger(),
                 "Publishing pose: [%.3f, %.3f, %.3f]",
                 pose_msg.pose.pose.position.x,
                 pose_msg.pose.pose.position.y,
                 pose_msg.pose.pose.position.z);

    pose_pub_->publish(pose_msg);
    seq_++;
}