#include "communication/husky_comms.h"
#include <boost/timer/timer.hpp>

HuskyComms::HuskyComms(std::shared_ptr<rclcpp::Node> nh, husky_inekf::husky_data_t *husky_data_buffer)
    : nh_(nh), husky_data_buffer_(husky_data_buffer)
{
    // 声明并获取参数
    declare_parameters();
    get_parameters();

    // 创建订阅者
    create_subscribers();

    RCLCPP_INFO(nh_->get_logger(), "HuskyComms node initialized");
    RCLCPP_INFO(nh_->get_logger(), "Subscribing to IMU: %s", imu_topic_.c_str());
    RCLCPP_INFO(nh_->get_logger(), "Subscribing to Joint: %s", joint_topic_.c_str());
}

HuskyComms::~HuskyComms()
{
    RCLCPP_INFO(nh_->get_logger(), "HuskyComms node shutting down");
}

void HuskyComms::declare_parameters()
{
    rcl_interfaces::msg::ParameterDescriptor imu_topic_desc;
    imu_topic_desc.description = "IMU数据话题名称";
    imu_topic_desc.additional_constraints = "必须是有效的ROS话题名称";
    nh_->declare_parameter("settings.imu_topic", "/imu/data", imu_topic_desc);

    // 关节状态话题参数
    rcl_interfaces::msg::ParameterDescriptor joint_topic_desc;
    joint_topic_desc.description = "关节状态话题名称，用于获取车轮编码器数据";
    joint_topic_desc.additional_constraints = "必须发布sensor_msgs/JointState消息";
    nh_->declare_parameter("settings.joint_topic", "/wheel_states", joint_topic_desc);

    // 车轮半径参数
    rcl_interfaces::msg::ParameterDescriptor wheel_radius_desc;
    wheel_radius_desc.description = "车轮半径 (米)";
    wheel_radius_desc.floating_point_range.resize(1);
    wheel_radius_desc.floating_point_range[0].from_value = 0.05; // 最小5cm
    wheel_radius_desc.floating_point_range[0].to_value = 1.0;    // 最大1m
    wheel_radius_desc.floating_point_range[0].step = 0.001;      // 步长1mm
    nh_->declare_parameter("settings.wheel_radius", 0.15, wheel_radius_desc);

    // 车辆轮距参数
    rcl_interfaces::msg::ParameterDescriptor track_width_desc;
    track_width_desc.description = "车辆轮距，左右车轮中心距离 (米)";
    track_width_desc.floating_point_range.resize(1);
    track_width_desc.floating_point_range[0].from_value = 0.1; // 最小10cm
    track_width_desc.floating_point_range[0].to_value = 3.0;   // 最大3m
    track_width_desc.floating_point_range[0].step = 0.001;     // 步长1mm
    nh_->declare_parameter("settings.vehicle_track_width", 0.412, track_width_desc);

    // 车辆长度参数
    rcl_interfaces::msg::ParameterDescriptor vehicle_length_desc;
    vehicle_length_desc.description = "车辆长度，前后轴距离 (米)";
    vehicle_length_desc.floating_point_range.resize(1);
    vehicle_length_desc.floating_point_range[0].from_value = 0.0; // 可以为0（差分驱动）
    vehicle_length_desc.floating_point_range[0].to_value = 5.0;   // 最大5m
    vehicle_length_desc.floating_point_range[0].step = 0.001;     // 步长1mm
    nh_->declare_parameter("settings.vehicle_length", 0.0, vehicle_length_desc);

    // 车轮速度更新使能参数
    rcl_interfaces::msg::ParameterDescriptor wheel_vel_enable_desc;
    wheel_vel_enable_desc.description = "是否启用车轮编码器速度更新";
    wheel_vel_enable_desc.additional_constraints = "推荐启用以获得更准确的里程计";
    nh_->declare_parameter("settings.enable_wheel_velocity_update", true, wheel_vel_enable_desc);

    // 车轮速度话题参数
    rcl_interfaces::msg::ParameterDescriptor wheel_vel_topic_desc;
    wheel_vel_topic_desc.description = "车轮速度数据话题名称";
    wheel_vel_topic_desc.additional_constraints = "通常与joint_topic相同";
    nh_->declare_parameter("settings.wheel_velocity_topic", "/joint_states", wheel_vel_topic_desc);

    // 相机速度更新使能参数
    rcl_interfaces::msg::ParameterDescriptor camera_vel_enable_desc;
    camera_vel_enable_desc.description = "是否启用相机视觉里程计速度更新";
    camera_vel_enable_desc.additional_constraints = "需要视觉SLAM系统支持";
    nh_->declare_parameter("settings.enable_camera_velocity_update", false, camera_vel_enable_desc);

    // 相机速度话题参数
    rcl_interfaces::msg::ParameterDescriptor camera_vel_topic_desc;
    camera_vel_topic_desc.description = "相机视觉里程计话题名称";
    camera_vel_topic_desc.additional_constraints = "必须发布nav_msgs/Odometry消息";
    nh_->declare_parameter("settings.camera_velocity_topic", "/zed_node/odom", camera_vel_topic_desc);

    // GPS速度更新使能参数
    rcl_interfaces::msg::ParameterDescriptor gps_vel_enable_desc;
    gps_vel_enable_desc.description = "是否启用GPS速度更新";
    gps_vel_enable_desc.additional_constraints = "适用于室外环境,需要GPS信号良好";
    nh_->declare_parameter("settings.enable_gps_velocity_update", false, gps_vel_enable_desc);

    // GPS速度话题参数
    rcl_interfaces::msg::ParameterDescriptor gps_vel_topic_desc;
    gps_vel_topic_desc.description = "GPS速度数据话题名称";
    gps_vel_topic_desc.additional_constraints = "必须发布geometry_msgs/TwistWithCovarianceStamped消息";
    nh_->declare_parameter("settings.gps_velocity_topic", "/gps/vel", gps_vel_topic_desc);

    // 相机到车体的平移变换参数
    rcl_interfaces::msg::ParameterDescriptor cam_translation_desc;
    cam_translation_desc.description = "相机相对于车体坐标系的平移变换 [x, y, z] (米)";
    cam_translation_desc.additional_constraints = "格式: [x, y, z],右手坐标系,x向前,y向左,z向上";
    std::vector<double> default_translation = {0.0, 0.0, 0.0};
    nh_->declare_parameter("settings.translation_cam_body", default_translation, cam_translation_desc);

    // 相机到车体的旋转变换参数（四元数）
    rcl_interfaces::msg::ParameterDescriptor cam_rotation_desc;
    cam_rotation_desc.description = "相机相对于车体坐标系的旋转变换四元数 [w, x, y, z]";
    cam_rotation_desc.additional_constraints = "格式: [w, x, y, z]，单位四元数，模长为1";
    std::vector<double> default_cam_rotation = {1.0, 0.0, 0.0, 0.0}; // 单位四元数
    nh_->declare_parameter("settings.rotation_cam_body", default_cam_rotation, cam_rotation_desc);

    // IMU到车体的旋转变换参数（四元数）
    rcl_interfaces::msg::ParameterDescriptor imu_rotation_desc;
    imu_rotation_desc.description = "IMU相对于车体坐标系的旋转变换四元数 [w, x, y, z]";
    imu_rotation_desc.additional_constraints = "格式: [w, x, y, z],单位四元数,根据IMU安装方向设定";
    std::vector<double> default_imu_rotation = {0.0, 0.7071, -0.7071, 0.0}; // 90度旋转示例
    nh_->declare_parameter("settings.rotation_imu_body", default_imu_rotation, imu_rotation_desc);
}

void HuskyComms::get_parameters()
{
    // 获取话题参数
    imu_topic_ = nh_->get_parameter("settings.imu_topic").as_string();
    joint_topic_ = nh_->get_parameter("settings.joint_topic").as_string();
    RCLCPP_INFO(nh_->get_logger(), "subscribe imu topic: %s", imu_topic_);

    // 获取物理参数
    wheel_radius_ = nh_->get_parameter("settings.wheel_radius").as_double();
    vehicle_track_width_ = nh_->get_parameter("settings.vehicle_track_width").as_double();
    vehicle_length_ = nh_->get_parameter("settings.vehicle_length").as_double();

    // 获取速度类型参数
    enable_wheel_vel_ = nh_->get_parameter("settings.enable_wheel_velocity_update").as_bool();
    wheel_vel_topic_ = nh_->get_parameter("settings.wheel_velocity_topic").as_string();
    enable_camera_vel_ = nh_->get_parameter("settings.enable_camera_velocity_update").as_bool();
    camera_vel_topic_ = nh_->get_parameter("settings.camera_velocity_topic").as_string();
    enable_gps_vel_ = nh_->get_parameter("settings.enable_gps_velocity_update").as_bool();
    gps_vel_topic_ = nh_->get_parameter("settings.gps_velocity_topic").as_string();

    // 获取变换参数
    auto translation_cam_body = nh_->get_parameter("settings.translation_cam_body").as_double_array();
    auto rotation_cam_body = nh_->get_parameter("settings.rotation_cam_body").as_double_array();
    rotation_imu_body_ = nh_->get_parameter("settings.rotation_imu_body").as_double_array();

    // 设置相机到车体的变换矩阵
    cam_to_body_ = Eigen::Matrix4d::Identity();

    if (rotation_cam_body.size() >= 4)
    {
        Eigen::Quaternion<double> orientation_quat(rotation_cam_body[0],
                                                   rotation_cam_body[1],
                                                   rotation_cam_body[2],
                                                   rotation_cam_body[3]);
        cam_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
    }

    if (translation_cam_body.size() >= 3)
    {
        cam_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(translation_cam_body[0],
                                                         translation_cam_body[1],
                                                         translation_cam_body[2]);
    }
}

void HuskyComms::create_subscribers()
{
    // IMU订阅者
    imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 1000,
        std::bind(&HuskyComms::imuCallback, this, std::placeholders::_1));

    // 根据配置创建不同的订阅者
    if (enable_wheel_vel_)
    {
        wheel_vel_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
            wheel_vel_topic_, 1000,
            std::bind(&HuskyComms::jointStateVelocityCallback, this, std::placeholders::_1));
    }
    else
    {
        joint_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
            joint_topic_, 1000,
            std::bind(&HuskyComms::jointStateCallback, this, std::placeholders::_1));
    }

    // 相机里程计订阅者
    if (enable_camera_vel_)
    {
        cam_vel_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
            camera_vel_topic_, 1000,
            std::bind(&HuskyComms::CameraOdomCallBack, this, std::placeholders::_1));
    }

    // GPS速度订阅者
    if (enable_gps_vel_)
    {
        gps_vel_sub_ = nh_->create_subscription<geometry_msgs::msg::TwistStamped>(
            gps_vel_topic_, 1000,
            std::bind(&HuskyComms::GPSvelocityCallback, this, std::placeholders::_1));
    }
}

void HuskyComms::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    RCLCPP_DEBUG(nh_->get_logger(), "Received IMU data");

    auto imu_ptr =
        std::make_shared<husky_inekf::ImuMeasurement<double>>(*imu_msg, rotation_imu_body_);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    husky_data_buffer_->imu_q.push(imu_ptr);
}

void HuskyComms::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
{
    RCLCPP_DEBUG(nh_->get_logger(), "Received joint state data");

    auto joint_ptr =
        std::make_shared<husky_inekf::JointStateMeasurement>(*joint_msg, 4, wheel_radius_,
                                                             vehicle_track_width_, vehicle_length_);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    husky_data_buffer_->joint_state_q.push(joint_ptr);
}

void HuskyComms::jointStateVelocityCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
{
    RCLCPP_DEBUG(nh_->get_logger(), "Received joint state velocity data");

    auto joint_state_ptr = std::make_shared<husky_inekf::JointStateMeasurement>(
        *joint_msg, 4, wheel_radius_, vehicle_track_width_, vehicle_length_);

    // 设置速度消息
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header = joint_msg->header;

    vel_msg.twist.linear.x = joint_state_ptr->getBodyLinearVelocity()(0);
    vel_msg.twist.linear.y = joint_state_ptr->getBodyLinearVelocity()(1);
    vel_msg.twist.linear.z = joint_state_ptr->getBodyLinearVelocity()(2);

    vel_msg.twist.angular.x = joint_state_ptr->getBodyAngularVelocity()(0);
    vel_msg.twist.angular.y = joint_state_ptr->getBodyAngularVelocity()(1);
    vel_msg.twist.angular.z = joint_state_ptr->getBodyAngularVelocity()(2);

    auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(vel_msg);

    // 分别加锁存储数据
    {
        std::lock_guard<std::mutex> lock2(husky_data_buffer_->joint_state_mutex);
        husky_data_buffer_->joint_state_q.push(joint_state_ptr);
    }

    {
        std::lock_guard<std::mutex> lock(husky_data_buffer_->wheel_vel_mutex);
        husky_data_buffer_->wheel_velocity_q.push(vel_ptr);
    }
}

void HuskyComms::GPSvelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg)
{
    RCLCPP_DEBUG(nh_->get_logger(), "Received GPS velocity data");

    auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(*vel_msg);
    std::lock_guard<std::mutex> lock(husky_data_buffer_->gps_vel_mutex);
    husky_data_buffer_->gps_velocity_q.push(vel_ptr);
}

void HuskyComms::CameraOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr camera_odom_msg)
{
    RCLCPP_DEBUG(nh_->get_logger(), "Received camera odometry data");

    auto camera_odom_ptr = std::make_shared<husky_inekf::CameraOdomMeasurement>(*camera_odom_msg);

    // 需要两个里程计数据来计算速度
    if (husky_data_buffer_->camera_odom_q.empty())
    {
        husky_data_buffer_->camera_odom_q.push(camera_odom_ptr);
        return;
    }

    auto prev_transformation =
        husky_data_buffer_->camera_odom_q.front().get()->getTransformation();
    double prev_time = husky_data_buffer_->camera_odom_q.front().get()->getTime();

    husky_data_buffer_->camera_odom_q.pop();

    // 插入当前相机里程计数据
    husky_data_buffer_->camera_odom_q.push(camera_odom_ptr);
    auto curr_transformation = camera_odom_ptr->getTransformation();
    double curr_time = camera_odom_ptr->getTime();

    double time_diff = curr_time - prev_time;

    Eigen::Matrix4d transformation = cam_to_body_.inverse() * prev_transformation.inverse() *
                                     curr_transformation * cam_to_body_;
    Eigen::Matrix4d twist_se3 = transformation.log();

    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header = camera_odom_msg->header;

    vel_msg.twist.linear.x = twist_se3(0, 3) / time_diff;
    vel_msg.twist.linear.y = twist_se3(1, 3) / time_diff;
    vel_msg.twist.linear.z = twist_se3(2, 3) / time_diff;

    vel_msg.twist.angular.x = twist_se3(2, 1) / time_diff;
    vel_msg.twist.angular.y = -twist_se3(2, 0) / time_diff;
    vel_msg.twist.angular.z = twist_se3(1, 0) / time_diff;

    auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(vel_msg);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->cam_vel_mutex);
    husky_data_buffer_->camera_velocity_q.push(vel_ptr);
}