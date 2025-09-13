/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   path_publisher_node.cpp
 *  @author Ross Hartley
 *  @brief  Subscribes to pose and publish a path (ROS2 version)
 *  @date   March 20, 2019
 **/

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <mutex>
#include <fstream>
#include <thread>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPublisherNode : public rclcpp::Node
{
public:
    PathPublisherNode() : Node("path_publisher_node")
    {
        // 声明参数
        declare_parameters();
        get_parameters();

        RCLCPP_INFO(this->get_logger(), "pose_topic: %s, path_topic: %s",
                    pose_topic_.c_str(), path_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "path publish rate: %.1f", publish_rate_);

        // 等待第一个位姿消息来确定坐标系
        RCLCPP_INFO(this->get_logger(), "Waiting for first pose message...");

        // 创建订阅者和发布者
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, 1000,
            std::bind(&PathPublisherNode::poseCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

        // 启动路径发布线程
        path_publishing_thread_ = std::thread([this]
                                              { this->pathPublishingThread(); });

        RCLCPP_INFO(this->get_logger(), "PathPublisherNode initialized");
    }

    ~PathPublisherNode()
    {
        if (path_publishing_thread_.joinable())
        {
            path_publishing_thread_.join();
        }
    }

private:
    void declare_parameters()
    {
        this->declare_parameter("pose_topic", "/husky/inekf_estimation/pose");
        this->declare_parameter("path_topic", "/husky/inekf_estimation/path");
        this->declare_parameter("publish_rate", 1000.0);
        this->declare_parameter("pose_skip", 100);
    }

    void get_parameters()
    {
        pose_topic_ = this->get_parameter("pose_topic").as_string();
        path_topic_ = this->get_parameter("path_topic").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        pose_skip_ = this->get_parameter("pose_skip").as_int();
    }

    // 位姿消息回调
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // 如果是第一个消息，设置坐标系
        if (pose_frame_.empty())
        {
            pose_frame_ = msg->header.frame_id;
            RCLCPP_INFO(this->get_logger(), "Set pose frame to: %s", pose_frame_.c_str());
        }

        // 跳过某些位姿（降采样）
        static uint32_t pose_count = 0;
        if (pose_skip_ > 0 && (pose_count % pose_skip_) != 0)
        {
            pose_count++;
            return;
        }
        pose_count++;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        RCLCPP_DEBUG(this->get_logger(),
                     "Adding pose to path: [%.3f, %.3f, %.3f]",
                     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        std::lock_guard<std::mutex> lock(poses_mutex_);
        poses_.push_back(pose);
    }

    void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose;

        std::lock_guard<std::mutex> lock(poses_mutex_);
        poses_.push_back(pose);
    }

    // 发布路径
    void pathPublish()
    {
        std::lock_guard<std::mutex> lock(poses_mutex_);

        if (poses_.empty())
        {
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = pose_frame_;
        path_msg.poses = poses_;

        RCLCPP_DEBUG(this->get_logger(),
                     "Publishing path with %zu poses",
                     path_msg.poses.size());

        path_pub_->publish(path_msg);
        seq_++;
    }

    // 路径发布线程
    void pathPublishingThread()
    {
        rclcpp::Rate loop_rate(publish_rate_);
        while (rclcpp::ok())
        {
            pathPublish();
            loop_rate.sleep();
        }
    }

    // ROS2组件
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // 参数
    std::string pose_topic_;
    std::string path_topic_;
    std::string pose_frame_;
    double publish_rate_;
    int pose_skip_;

    // 状态
    uint32_t seq_ = 0;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    std::mutex poses_mutex_;
    std::thread path_publishing_thread_;
};

int main(int argc, char **argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}