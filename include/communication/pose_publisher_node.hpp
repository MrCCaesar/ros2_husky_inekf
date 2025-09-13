#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sstream>
#include <fstream>
#include <array>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include "system/husky_state.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class PosePublisherNode : public rclcpp::Node
{
public:
    PosePublisherNode();
    ~PosePublisherNode();

    // 发布位姿
    void posePublish(const husky_inekf::HuskyState &state_);

private:
    void declare_parameters();
    void get_parameters();

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    std::string pose_topic_;
    std::string pose_frame_;
    uint32_t seq_ = 0;
    double publish_rate_;
    int pose_skip_;

    std::queue<std::array<float, 3>> pose_from_csv_;
    std::array<float, 3> first_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    std::mutex poses_mutex_;
    std::thread pose_publishing_thread_;
};