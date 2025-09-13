#include "system/husky_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/timer/timer.hpp>

#include <vector>
#include <numeric>

HuskySystem::HuskySystem(rclcpp::Node::SharedPtr node, husky_inekf::husky_data_t *husky_data_buffer) : node_(node),
                                                                                                       husky_data_buffer_(husky_data_buffer),
                                                                                                       estimator_(node),
                                                                                                       new_pose_ready_(false),
                                                                                                       logger_(node_->get_logger()),
                                                                                                       clock_(node_->get_clock())
{

    // initialize velocity noise
    // TODO: integrate them into noiseParams
    declare_parameters();
    get_parameters();

    outfile_.open(file_name_, std::ofstream::out);
    vel_est_outfile_.open(vel_est_file_name_, std::ofstream::out);
    bias_est_outfile_.open(bias_est_file_name_, std::ofstream::out);
    outfile_.precision(20);
    vel_est_outfile_.precision(20);
    bias_est_outfile_.precision(20);

    if (enable_debug_logger_)
    {
        vel_input_outfile_.open(vel_input_file_name_, std::ofstream::out);
        imu_outfile_.open(imu_file_name_, std::ofstream::out);
        vel_input_outfile_.precision(20);
        imu_outfile_.precision(20);
    }

    last_imu_time_ = 0;
    skip_count_ = 0;
}

HuskySystem::~HuskySystem()
{
    RCLCPP_INFO(logger_, "Ready to close Husky system");
    outfile_.close();
    vel_est_outfile_.close();
    bias_est_outfile_.close();
    vel_input_outfile_.close();
    imu_outfile_.close();
}

void HuskySystem::declare_parameters()
{
    RCLCPP_INFO(node_->get_logger(), "声明速度噪声参数...");

    // 车轮速度标准差
    rcl_interfaces::msg::ParameterDescriptor wheel_vel_std_desc;
    wheel_vel_std_desc.description = "车轮速度测量噪声标准差 (m/s)";
    wheel_vel_std_desc.additional_constraints = "用于构建3x3对角协方差矩阵，影响车轮里程计的置信度";
    wheel_vel_std_desc.floating_point_range.resize(1);
    wheel_vel_std_desc.floating_point_range[0].from_value = 0.001; // 1mm/s
    wheel_vel_std_desc.floating_point_range[0].to_value = 1.0;     // 1m/s
    wheel_vel_std_desc.floating_point_range[0].step = 0.001;
    node_->declare_parameter("noise.wheel_vel_std", 0.05, wheel_vel_std_desc);

    // 相机速度标准差
    rcl_interfaces::msg::ParameterDescriptor camera_vel_std_desc;
    camera_vel_std_desc.description = "相机视觉里程计速度测量噪声标准差 (m/s)";
    camera_vel_std_desc.additional_constraints = "视觉SLAM或VO系统的速度估计不确定性,光照条件和纹理影响此值";
    camera_vel_std_desc.floating_point_range.resize(1);
    camera_vel_std_desc.floating_point_range[0].from_value = 0.001; // 1mm/s
    camera_vel_std_desc.floating_point_range[0].to_value = 2.0;     // 2m/s
    camera_vel_std_desc.floating_point_range[0].step = 0.001;
    node_->declare_parameter("noise.camera_vel_std", 0.05, camera_vel_std_desc);

    // GPS速度标准差
    rcl_interfaces::msg::ParameterDescriptor gps_vel_std_desc;
    gps_vel_std_desc.description = "GPS速度测量噪声标准差 (m/s)";
    gps_vel_std_desc.additional_constraints = "GPS接收器的速度精度,受信号质量、多径效应和接收器质量影响";
    gps_vel_std_desc.floating_point_range.resize(1);
    gps_vel_std_desc.floating_point_range[0].from_value = 0.01; // 1cm/s
    gps_vel_std_desc.floating_point_range[0].to_value = 5.0;    // 5m/s
    gps_vel_std_desc.floating_point_range[0].step = 0.01;
    node_->declare_parameter("noise.gps_vel_std", 0.05, gps_vel_std_desc);

    // 位姿发布使能
    rcl_interfaces::msg::ParameterDescriptor pose_pub_desc;
    pose_pub_desc.description = "启用位姿话题发布";
    pose_pub_desc.additional_constraints = "发布geometry_msgs/PoseStamped话题，用于可视化和下游节点";
    node_->declare_parameter("settings.enable_pose_publisher", false, pose_pub_desc);

    // 位姿日志使能
    rcl_interfaces::msg::ParameterDescriptor pose_log_desc;
    pose_log_desc.description = "启用位姿数据日志记录";
    pose_log_desc.additional_constraints = "将位姿数据保存到文件，用于离线分析和轨迹评估";
    node_->declare_parameter("settings.enable_pose_logger", false, pose_log_desc);

    // 调试日志使能
    rcl_interfaces::msg::ParameterDescriptor debug_log_desc;
    debug_log_desc.description = "启用详细调试日志记录";
    debug_log_desc.additional_constraints = "记录内部状态变量，用于算法调试和性能分析，影响性能";
    node_->declare_parameter("settings.enable_debug_logger", false, debug_log_desc);

    // 位姿日志跳帧数
    rcl_interfaces::msg::ParameterDescriptor pose_skip_desc;
    pose_skip_desc.description = "位姿日志记录的跳帧数";
    pose_skip_desc.additional_constraints = "每N帧记录一次位姿，减少文件大小，100表示每100帧记录一次";
    pose_skip_desc.integer_range.resize(1);
    pose_skip_desc.integer_range[0].from_value = 1;  // 最小1帧
    pose_skip_desc.integer_range[0].to_value = 1000; // 最大1000帧
    pose_skip_desc.integer_range[0].step = 1;
    node_->declare_parameter("settings.log_pose_skip", 100, pose_skip_desc);

    // InEKF位姿文件名
    rcl_interfaces::msg::ParameterDescriptor pose_file_desc;
    pose_file_desc.description = "InEKF位姿估计输出文件名";
    pose_file_desc.additional_constraints = "保存位姿轨迹的文件，格式: timestamp x y z qx qy qz qw";
    node_->declare_parameter("settings.inekf_pose_filename",
                             std::string("husky_inekf_pose.txt"), pose_file_desc);

    // 速度估计文件名
    rcl_interfaces::msg::ParameterDescriptor vel_est_file_desc;
    vel_est_file_desc.description = "速度估计输出文件名";
    vel_est_file_desc.additional_constraints = "保存估计的速度数据，格式: timestamp vx vy vz";
    node_->declare_parameter("settings.inekf_vel_est_file_name",
                             std::string("vel_est.txt"), vel_est_file_desc);

    // 偏置估计文件名
    rcl_interfaces::msg::ParameterDescriptor bias_est_file_desc;
    bias_est_file_desc.description = "IMU偏置估计输出文件名";
    bias_est_file_desc.additional_constraints = "保存估计的IMU偏置，格式: timestamp gyro_bias_x gyro_bias_y gyro_bias_z accel_bias_x accel_bias_y accel_bias_z";
    node_->declare_parameter("settings.inekf_bias_est_file_name",
                             std::string("bias_est.txt"), bias_est_file_desc);

    // 速度输入文件名
    rcl_interfaces::msg::ParameterDescriptor vel_input_file_desc;
    vel_input_file_desc.description = "速度输入数据文件名";
    vel_input_file_desc.additional_constraints = "保存各传感器的速度输入，用于调试和分析";
    node_->declare_parameter("settings.inekf_vel_input_file_name",
                             std::string("vel_input.txt"), vel_input_file_desc);

    // IMU数据文件名
    rcl_interfaces::msg::ParameterDescriptor imu_file_desc;
    imu_file_desc.description = "IMU原始数据文件名";
    imu_file_desc.additional_constraints = "保存IMU原始数据，格式: timestamp ax ay az gx gy gz";
    node_->declare_parameter("settings.inekf_imu_file_name",
                             std::string("imu.txt"), imu_file_desc);
}

void HuskySystem::get_parameters()
{
    RCLCPP_INFO(node_->get_logger(), "初始化System...");

    // 车轮速度协方差矩阵
    double wheel_std = node_->get_parameter("noise.wheel_vel_std").as_double();
    wheel_vel_cov_ = wheel_std * wheel_std * Eigen::Matrix<double, 3, 3>::Identity();

    // 相机速度协方差矩阵
    double camera_std = node_->get_parameter("noise.camera_vel_std").as_double();
    camera_vel_cov_ = camera_std * camera_std * Eigen::Matrix<double, 3, 3>::Identity();

    // GPS速度协方差矩阵
    double gps_std = node_->get_parameter("noise.gps_vel_std").as_double();
    gps_vel_cov_ = gps_std * gps_std * Eigen::Matrix<double, 3, 3>::Identity();

    // 速度更新方法设置
    enable_wheel_vel_ = node_->get_parameter("settings.enable_wheel_velocity_update").as_bool();
    enable_camera_vel_ = node_->get_parameter("settings.enable_camera_velocity_update").as_bool();
    enable_gps_vel_ = node_->get_parameter("settings.enable_gps_velocity_update").as_bool();

    // 输出和日志设置
    enable_pose_publisher_ = node_->get_parameter("settings.enable_pose_publisher").as_bool();
    enable_pose_logger_ = node_->get_parameter("settings.enable_pose_logger").as_bool();
    enable_debug_logger_ = node_->get_parameter("settings.enable_debug_logger").as_bool();
    log_pose_skip_ = node_->get_parameter("settings.log_pose_skip").as_int();

    // 文件名设置
    file_name_ = node_->get_parameter("settings.inekf_pose_filename").as_string();
    vel_est_file_name_ = node_->get_parameter("settings.inekf_vel_est_file_name").as_string();
    bias_est_file_name_ = node_->get_parameter("settings.inekf_bias_est_file_name").as_string();
    vel_input_file_name_ = node_->get_parameter("settings.inekf_vel_input_file_name").as_string();
    imu_file_name_ = node_->get_parameter("settings.inekf_imu_file_name").as_string();

    RCLCPP_INFO(node_->get_logger(), "系统设置初始化完成");
}
void HuskySystem::step()
{

    // if the estimator is initialized
    if (estimator_.enabled())
    {

        // if IMU measurement exists we do prediction
        if (updateNextIMU())
        {
            estimator_.propagateIMU(*(imu_packet_.get()), state_);
            if (enable_debug_logger_)
            {
                imu_outfile_ << imu_packet_->getTime() << " "
                             << imu_packet_->angular_velocity.x << " "
                             << imu_packet_->angular_velocity.y << " "
                             << imu_packet_->angular_velocity.z << " "
                             << imu_packet_->linear_acceleration.x << " "
                             << imu_packet_->linear_acceleration.y << " "
                             << imu_packet_->linear_acceleration.z << std::endl
                             << std::flush;
            }
            new_pose_ready_ = true;
        }

        // update using body velocity from wheel encoders
        if (enable_wheel_vel_ && updateNextWheelVelocity())
        {
            estimator_.correctVelocity(*(wheel_velocity_packet_.get()), state_, wheel_vel_cov_);
            new_pose_ready_ = true;

            // record
            auto v_in = wheel_velocity_packet_->getLinearVelocity();
            if (enable_debug_logger_)
            {
                vel_input_outfile_ << wheel_velocity_packet_->getTime() << " "
                                   << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl
                                   << std::flush;
            }
        }

        // update using camera velocity
        if (enable_camera_vel_ && updateNextCameraVelocity())
        {
            estimator_.correctVelocity(*(camera_velocity_packet_.get()), state_, camera_vel_cov_);
            new_pose_ready_ = true;

            // record
            auto v_in = camera_velocity_packet_->getLinearVelocity();
            if (enable_debug_logger_)
            {
                vel_input_outfile_ << camera_velocity_packet_->getTime() << " "
                                   << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl
                                   << std::flush;
            }
        }

        // update using gps velocity
        if (enable_gps_vel_ && updateNextGPSVelocity())
        {
            estimator_.correctVelocity(*(gps_velocity_packet_.get()), state_, gps_vel_cov_);
            new_pose_ready_ = true;

            // record
            auto v_in = gps_velocity_packet_->getLinearVelocity();
            if (enable_debug_logger_)
            {
                vel_input_outfile_ << gps_velocity_packet_->getTime() << " "
                                   << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl
                                   << std::flush;
            }
        }

        if (enable_pose_publisher_ && new_pose_ready_)
        {
            pose_publisher_node_.posePublish(state_);
        }

        if (enable_pose_logger_ && new_pose_ready_)
        {
            logPoseTxt(state_);
        }

        new_pose_ready_ = false;
    }
    // initialization
    else
    {
        if (estimator_.biasInitialized())
        {
            // wait until we receive imu msg
            while (!updateNextIMU())
            {
            };

            if (enable_wheel_vel_)
            {
                while (!updateNextWheelVelocity())
                {
                }
                estimator_.initState(*(imu_packet_.get()), *(wheel_velocity_packet_.get()), state_);
            }
            else if (enable_camera_vel_)
            {
                while (!updateNextCameraVelocity())
                {
                }
                estimator_.initState(*(imu_packet_.get()), *(camera_velocity_packet_.get()), state_);
            }
            else if (enable_gps_vel_)
            {
                while (!updateNextGPSVelocity())
                {
                }
                estimator_.initState(*(imu_packet_.get()), *(gps_velocity_packet_.get()), state_);
            }

            estimator_.enableFilter();
            husky_data_buffer_->wheel_velocity_q = {};
            husky_data_buffer_->camera_velocity_q = {};
            husky_data_buffer_->gps_velocity_q = {};

            RCLCPP_INFO(logger_, "State initialized.");
        }
        else
        {
            while (!updateNextIMU())
            {
            };
            estimator_.initBias(*(imu_packet_.get()));
        }
    }
}

void HuskySystem::logPoseTxt(const husky_inekf::HuskyState &state_)
{
    if (skip_count_ == 0)
    {
        double t = state_.getTime();

        // log pose tum style
        outfile_ << t << " " << state_.x() << " " << state_.y() << " " << state_.z() << " "
                 << state_.getQuaternion().x() << " " << state_.getQuaternion().y() << " "
                 << state_.getQuaternion().z() << " " << state_.getQuaternion().w()
                 << std::endl
                 << std::flush;

        // log estimated velocity
        auto vel_est = state_.getWorldVelocity();
        vel_est_outfile_ << t << " " << vel_est(0) << " " << vel_est(1) << " "
                         << vel_est(2) << std::endl
                         << std::flush;

        // log estimated bias
        auto bias_est = state_.getImuBias();
        bias_est_outfile_ << t << " " << bias_est(0) << " " << bias_est(1) << " "
                          << bias_est(2) << " " << bias_est(3) << " " << bias_est(4) << " "
                          << bias_est(5) << std::endl
                          << std::flush;

        skip_count_ = log_pose_skip_;
    }
    else
    {
        skip_count_--;
    }
}

// Private Functions
bool HuskySystem::updateNextIMU()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    if (!husky_data_buffer_->imu_q.empty())
    {

        if (husky_data_buffer_->imu_q.size() > 1)
        {
            RCLCPP_INFO_STREAM(logger_, "Filter not running in real-time!");
            RCLCPP_INFO_STREAM(logger_, "IMU queue size: " << husky_data_buffer_->imu_q.size());
        }
        imu_packet_ = husky_data_buffer_->imu_q.front();
        husky_data_buffer_->imu_q.pop();
        // Update Husky State
        state_.setImu(imu_packet_);

        return true;
    }
    return false;
}

bool HuskySystem::updateNextJointState()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    if (!husky_data_buffer_->joint_state_q.empty())
    {

        if (husky_data_buffer_->joint_state_q.size() > 1)
        {
            RCLCPP_INFO_STREAM(logger_, "Filter not running in real-time!");
            RCLCPP_INFO_STREAM(logger_, "Joint state queue size: " << husky_data_buffer_->joint_state_q.size());
        }

        joint_state_packet_ = husky_data_buffer_->joint_state_q.front();
        husky_data_buffer_->joint_state_q.pop();

        // Update Husky State
        state_.setJointState(joint_state_packet_);

        return true;
    }
    return false;
}

bool HuskySystem::updateNextWheelVelocity()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->wheel_vel_mutex);

    if (!husky_data_buffer_->wheel_velocity_q.empty())
    {

        if (husky_data_buffer_->wheel_velocity_q.size() > 1)
        {
            RCLCPP_INFO_STREAM(logger_, "Filter not running in real-time!");
            RCLCPP_INFO_STREAM(logger_, "Velocity queue size: " << husky_data_buffer_->wheel_velocity_q.size());
        }

        wheel_velocity_packet_ = husky_data_buffer_->wheel_velocity_q.front();
        husky_data_buffer_->wheel_velocity_q.pop();

        return true;
    }
    return false;
}

bool HuskySystem::updateNextCameraVelocity()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->cam_vel_mutex);

    if (!husky_data_buffer_->camera_velocity_q.empty())
    {

        if (husky_data_buffer_->camera_velocity_q.size() > 1)
        {
            RCLCPP_INFO_STREAM(logger_, "Filter not running in real-time!");
            RCLCPP_INFO_STREAM(logger_, "Velocity queue size: " << husky_data_buffer_->camera_velocity_q.size());
        }

        camera_velocity_packet_ = husky_data_buffer_->camera_velocity_q.front();
        husky_data_buffer_->camera_velocity_q.pop();

        return true;
    }
    return false;
}

bool HuskySystem::updateNextGPSVelocity()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->gps_vel_mutex);

    if (!husky_data_buffer_->gps_velocity_q.empty())
    {

        if (husky_data_buffer_->gps_velocity_q.size() > 1)
        {
            RCLCPP_INFO_STREAM(logger_, "Filter not running in real-time!");
            RCLCPP_INFO_STREAM(logger_, "Velocity queue size: " << husky_data_buffer_->gps_velocity_q.size());
        }

        gps_velocity_packet_ = husky_data_buffer_->gps_velocity_q.front();
        husky_data_buffer_->gps_velocity_q.pop();

        return true;
    }
    return false;
}