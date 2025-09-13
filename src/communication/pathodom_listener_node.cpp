#include <rclcpp/rclcpp.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <limits>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class PathOdomListenerNode : public rclcpp::Node
{
public:
  PathOdomListenerNode() : Node("path_odom_listener_node")
  {
    // 声明参数
    this->declare_parameter("output_file", "/tmp/odom_pose_tum.txt");
    this->declare_parameter("odom_topic", "/odometry/filtered/output");

    // 获取参数
    file_name_tum_ = this->get_parameter("output_file").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    // 清空输出文件
    std::ofstream tum_outfile(file_name_tum_);
    tum_outfile.close();

    // 创建订阅者
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 1000,
        std::bind(&PathOdomListenerNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Path Odom Listener Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output file: %s", file_name_tum_.c_str());
  }

private:
  void pathMapCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty())
    {
      return;
    }

    geometry_msgs::msg::Pose pose = msg->poses.back().pose;
    long double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    savePoseCallback(pose, timestamp);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::Pose pose = msg->pose.pose;
    geometry_msgs::msg::Point position = pose.position;

    RCLCPP_DEBUG(this->get_logger(),
                 "Received odometry data: [%.3f, %.3f, %.3f]",
                 position.x, position.y, position.z);

    long double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    savePoseCallback(pose, timestamp);
  }

  void savePoseCallback(const geometry_msgs::msg::Pose &pose, const long double timestamp)
  {
    RCLCPP_DEBUG(this->get_logger(), "Saving pose to file");

    // TUM格式输出
    std::ofstream tum_outfile(file_name_tum_, std::ofstream::out | std::ofstream::app);
    tum_outfile.precision(std::numeric_limits<double>::max_digits10);
    tum_outfile << timestamp << " "
                << pose.position.x << " "
                << pose.position.y << " "
                << pose.position.z << " "
                << pose.orientation.x << " "
                << pose.orientation.y << " "
                << pose.orientation.z << " "
                << pose.orientation.w << std::endl
                << std::flush;

    tum_outfile.close();
  }

  // ROS2组件
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 参数
  std::string file_name_tum_;
  std::string odom_topic_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathOdomListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}