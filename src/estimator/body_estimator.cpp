#include "estimator/body_estimator.hpp"

namespace husky_inekf
{

    BodyEstimator::BodyEstimator(rclcpp::Node::SharedPtr node) : node_(node), t_prev_(0), imu_prev_(Eigen::Matrix<double, 6, 1>::Zero())
    {
        // 声明并获取参数
        declare_parameters();
        get_parameters();

        // 设置噪声参数
        inekf::NoiseParams params;
        params.setGyroscopeNoise(gyroscope_std_);
        params.setAccelerometerNoise(accelerometer_std_);
        params.setGyroscopeBiasNoise(gyroscope_bias_std_);
        params.setAccelerometerBiasNoise(accelerometer_bias_std_);

        filter_.setNoiseParams(params);

        RCLCPP_INFO(node_->get_logger(), "Noise parameters are initialized to:");
        // 注意：这里需要实现 NoiseParams 的流输出操作符，或者分别打印各个参数
        RCLCPP_INFO(node_->get_logger(), "Gyroscope noise: %f", gyroscope_std_);
        RCLCPP_INFO(node_->get_logger(), "Accelerometer noise: %f", accelerometer_std_);
        RCLCPP_INFO(node_->get_logger(), "Gyroscope bias noise: %f", gyroscope_bias_std_);
        RCLCPP_INFO(node_->get_logger(), "Accelerometer bias noise: %f", accelerometer_bias_std_);

        // 加载偏置先验
        if (gyroscope_bias_prior_.size() >= 3)
        {
            bg0_ << gyroscope_bias_prior_[0], gyroscope_bias_prior_[1], gyroscope_bias_prior_[2];
        }
        if (accelerometer_bias_prior_.size() >= 3)
        {
            ba0_ << accelerometer_bias_prior_[0], accelerometer_bias_prior_[1], accelerometer_bias_prior_[2];
        }

        RCLCPP_INFO(node_->get_logger(), "BodyEstimator initialized successfully");
    }

    BodyEstimator::~BodyEstimator()
    {
        RCLCPP_INFO(node_->get_logger(), "BodyEstimator destructor called");
    }

    void BodyEstimator::declare_parameters()
    {
        RCLCPP_INFO(node_->get_logger(), "声明估计器设置参数...");

        // 估计器调试开关
        rcl_interfaces::msg::ParameterDescriptor debug_desc;
        debug_desc.description = "启用估计器调试输出模式";
        debug_desc.additional_constraints = "启用后会输出详细的估计过程信息，影响性能";
        node_->declare_parameter("settings.estimator_enable_debug", false, debug_desc);

        // 静态偏置初始化
        rcl_interfaces::msg::ParameterDescriptor static_bias_desc;
        static_bias_desc.description = "启用静态偏置初始化模式";
        static_bias_desc.additional_constraints = "机器人静止时估计IMU偏置,提高初始化精度";
        node_->declare_parameter("settings.static_bias_initialization", true, static_bias_desc);

        // 使用IMU方向估计初始化偏置
        rcl_interfaces::msg::ParameterDescriptor init_bias_desc;
        init_bias_desc.description = "使用IMU方向估计来初始化偏置";
        init_bias_desc.additional_constraints = "基于重力方向估计加速度计偏置,需要IMU静止放置";
        node_->declare_parameter("settings.init_bias_using_orientation_est_from_imu", false, init_bias_desc);

        // 速度时间阈值
        rcl_interfaces::msg::ParameterDescriptor velocity_threshold_desc;
        velocity_threshold_desc.description = "速度测量时间阈值 (秒)";
        velocity_threshold_desc.additional_constraints = "超过此时间的速度测量将被丢弃，防止过时数据影响估计";
        velocity_threshold_desc.floating_point_range.resize(1);
        velocity_threshold_desc.floating_point_range[0].from_value = 0.01; // 最小10ms
        velocity_threshold_desc.floating_point_range[0].to_value = 1.0;    // 最大1s
        velocity_threshold_desc.floating_point_range[0].step = 0.001;      // 步长1ms
        node_->declare_parameter("settings.velocity_time_threshold", 0.1, velocity_threshold_desc);

        // 陀螺仪标准差
        rcl_interfaces::msg::ParameterDescriptor gyro_std_desc;
        gyro_std_desc.description = "陀螺仪测量噪声标准差 (rad/s)";
        gyro_std_desc.additional_constraints = "表示陀螺仪角速度测量的不确定性，影响姿态估计精度";
        gyro_std_desc.floating_point_range.resize(1);
        gyro_std_desc.floating_point_range[0].from_value = 0.0001; // 0.0001 rad/s = 0.0057°/s
        gyro_std_desc.floating_point_range[0].to_value = 0.1;      // 0.1 rad/s = 5.73°/s
        gyro_std_desc.floating_point_range[0].step = 0.0001;
        node_->declare_parameter("noise.gyroscope_std", 0.01, gyro_std_desc);

        // 加速度计标准差
        rcl_interfaces::msg::ParameterDescriptor accel_std_desc;
        accel_std_desc.description = "加速度计测量噪声标准差 (m/s²)";
        accel_std_desc.additional_constraints = "表示加速度计测量的不确定性，影响速度和位置估计精度";
        accel_std_desc.floating_point_range.resize(1);
        accel_std_desc.floating_point_range[0].from_value = 0.001; // 0.001 m/s²
        accel_std_desc.floating_point_range[0].to_value = 1.0;     // 1.0 m/s²
        accel_std_desc.floating_point_range[0].step = 0.001;
        node_->declare_parameter("noise.accelerometer_std", 0.1, accel_std_desc);

        // 陀螺仪偏置标准差
        rcl_interfaces::msg::ParameterDescriptor gyro_bias_std_desc;
        gyro_bias_std_desc.description = "陀螺仪偏置随机游走标准差 (rad/s)";
        gyro_bias_std_desc.additional_constraints = "描述陀螺仪偏置随时间变化的不确定性，值越小表示偏置越稳定";
        gyro_bias_std_desc.floating_point_range.resize(1);
        gyro_bias_std_desc.floating_point_range[0].from_value = 1e-6; // 1e-6 rad/s
        gyro_bias_std_desc.floating_point_range[0].to_value = 0.001;  // 0.001 rad/s
        gyro_bias_std_desc.floating_point_range[0].step = 1e-6;
        node_->declare_parameter("noise.gyroscope_bias_std", 0.0001, gyro_bias_std_desc);

        // 加速度计偏置标准差
        rcl_interfaces::msg::ParameterDescriptor accel_bias_std_desc;
        accel_bias_std_desc.description = "加速度计偏置随机游走标准差 (m/s²)";
        accel_bias_std_desc.additional_constraints = "描述加速度计偏置随时间变化的不确定性，值越小表示偏置越稳定";
        accel_bias_std_desc.floating_point_range.resize(1);
        accel_bias_std_desc.floating_point_range[0].from_value = 1e-5; // 1e-5 m/s²
        accel_bias_std_desc.floating_point_range[0].to_value = 0.1;    // 0.1 m/s²
        accel_bias_std_desc.floating_point_range[0].step = 1e-5;
        node_->declare_parameter("noise.accelerometer_bias_std", 0.01, accel_bias_std_desc);

        // 陀螺仪偏置先验
        rcl_interfaces::msg::ParameterDescriptor gyro_bias_prior_desc;
        gyro_bias_prior_desc.description = "陀螺仪偏置先验值 [x, y, z] (rad/s)";
        gyro_bias_prior_desc.additional_constraints = "IMU三轴陀螺仪的初始偏置估计值，通常通过标定获得";
        std::vector<double> default_gyro_bias = {0.0, 0.0, 0.0};
        node_->declare_parameter("prior.gyroscope_bias", default_gyro_bias, gyro_bias_prior_desc);

        // 加速度计偏置先验
        rcl_interfaces::msg::ParameterDescriptor accel_bias_prior_desc;
        accel_bias_prior_desc.description = "加速度计偏置先验值 [x, y, z] (m/s²)";
        accel_bias_prior_desc.additional_constraints = "IMU三轴加速度计的初始偏置估计值，通常通过标定获得";
        std::vector<double> default_accel_bias = {0.0, 0.0, 0.0};
        node_->declare_parameter("prior.accelerometer_bias", default_accel_bias, accel_bias_prior_desc);
    }

    void BodyEstimator::get_parameters()
    {
        // 获取估计器设置参数
        estimator_debug_enabled_ = node_->get_parameter("settings.estimator_enable_debug").as_bool();
        static_bias_initialization_ = node_->get_parameter("settings.static_bias_initialization").as_bool();
        use_imu_ori_est_init_bias_ = node_->get_parameter("settings.init_bias_using_orientation_est_from_imu").as_bool();
        velocity_t_thres_ = node_->get_parameter("settings.velocity_time_threshold").as_double();

        // 获取噪声参数
        gyroscope_std_ = node_->get_parameter("noise.gyroscope_std").as_double();
        accelerometer_std_ = node_->get_parameter("noise.accelerometer_std").as_double();
        gyroscope_bias_std_ = node_->get_parameter("noise.gyroscope_bias_std").as_double();
        accelerometer_bias_std_ = node_->get_parameter("noise.accelerometer_bias_std").as_double();

        // 获取先验参数
        gyroscope_bias_prior_ = node_->get_parameter("prior.gyroscope_bias").as_double_array();
        accelerometer_bias_prior_ = node_->get_parameter("prior.accelerometer_bias").as_double_array();
    }

    bool BodyEstimator::biasInitialized() { return bias_initialized_; }
    bool BodyEstimator::enabled() { return enabled_; }
    void BodyEstimator::enableFilter() { enabled_ = true; }

    void BodyEstimator::propagateIMU(const ImuMeasurement<double> &imu_packet_in, HuskyState &state)
    {
        // 从初始机器人状态初始化偏置
        if (!bias_initialized_)
        {
            initBias(imu_packet_in);
        }

        // 提取当前IMU数据 [w;a]
        Eigen::Matrix<double, 6, 1> imu;
        imu << imu_packet_in.angular_velocity.x,
            imu_packet_in.angular_velocity.y,
            imu_packet_in.angular_velocity.z,
            imu_packet_in.linear_acceleration.x,
            imu_packet_in.linear_acceleration.y,
            imu_packet_in.linear_acceleration.z;
        double t = imu_packet_in.getTime();

        // 基于IMU和接触数据传播状态
        double dt = t - t_prev_;
        if (estimator_debug_enabled_)
        {
            RCLCPP_INFO(node_->get_logger(), "Tprev %0.6lf T %0.6lf dt %0.6lf", t_prev_, t, dt);
        }

        if (dt > 0)
            filter_.Propagate(imu_prev_, dt);

        // correctKinematics(state);

        /// TODO: 检查IMU捷联惯导模型是否正确
        inekf::RobotState estimate = filter_.getState();
        Eigen::Matrix3d R = estimate.getRotation();
        Eigen::Vector3d p = estimate.getPosition();
        Eigen::Vector3d v = estimate.getVelocity();
        Eigen::Vector3d bias = estimate.getTheta();
        state.setBaseRotation(R);
        state.setBasePosition(p);
        state.setBaseVelocity(v);
        state.setImuBias(bias);
        state.setTime(t);

        // 存储之前的IMU数据
        t_prev_ = t;
        imu_prev_ = imu;
        seq_ = imu_packet_in.header.seq;

        if (estimator_debug_enabled_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "IMU Propagation Complete: linacceleration x: %0.6f y: %.06f z: %0.6f",
                        imu_packet_in.linear_acceleration.x,
                        imu_packet_in.linear_acceleration.y,
                        imu_packet_in.linear_acceleration.z);
        }
    }

    // 修正速度
    void BodyEstimator::correctVelocity(const JointStateMeasurement &joint_state_packet_in, HuskyState &state, const Eigen::Matrix<double, 3, 3> &velocity_cov)
    {

        double t = joint_state_packet_in.getTime();

        if (std::abs(t - state.getTime()) < velocity_t_thres_)
        {
            Eigen::Vector3d measured_velocity = joint_state_packet_in.getBodyLinearVelocity();
            filter_.CorrectVelocity(measured_velocity, velocity_cov);

            inekf::RobotState estimate = filter_.getState();
            Eigen::Matrix3d R = estimate.getRotation();
            Eigen::Vector3d p = estimate.getPosition();
            Eigen::Vector3d v = estimate.getVelocity();
            Eigen::Vector3d bias = estimate.getTheta();

            state.setBaseRotation(R);
            state.setBasePosition(p);
            state.setBaseVelocity(v);
            state.setImuBias(bias);
            state.setTime(t);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Velocity not updated because of large time difference.");
            RCLCPP_DEBUG(node_->get_logger(), "t: %.20f", t);
            RCLCPP_DEBUG(node_->get_logger(), "state t: %.20f", state.getTime());
            RCLCPP_DEBUG(node_->get_logger(), "time diff: %.20f", t - state.getTime());
        }
    }

    void BodyEstimator::correctVelocity(const VelocityMeasurement &velocity_packet_in, HuskyState &state, const Eigen::Matrix<double, 3, 3> &velocity_cov)
    {

        double t = velocity_packet_in.getTime();

        if (std::abs(t - state.getTime()) < velocity_t_thres_)
        {
            Eigen::Vector3d measured_velocity = velocity_packet_in.getLinearVelocity();
            filter_.CorrectVelocity(measured_velocity, velocity_cov);

            inekf::RobotState estimate = filter_.getState();
            Eigen::Matrix3d R = estimate.getRotation();
            Eigen::Vector3d p = estimate.getPosition();
            Eigen::Vector3d v = estimate.getVelocity();
            Eigen::Vector3d bias = estimate.getTheta();

            Eigen::Vector3d gyro_bias = estimate.getGyroscopeBias();
            Eigen::Vector3d acc_bias = estimate.getAccelerometerBias();

            state.setBaseRotation(R);
            state.setBasePosition(p);
            state.setBaseVelocity(v);
            state.setImuBias(bias);
            state.setTime(t);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Velocity not updated because of large time difference.");
            RCLCPP_DEBUG(node_->get_logger(), "t: %.20f", t);
            RCLCPP_DEBUG(node_->get_logger(), "state t: %.20f", state.getTime());
            RCLCPP_DEBUG(node_->get_logger(), "time diff: %.20f", t - state.getTime());
        }
    }

    void BodyEstimator::initBias(const ImuMeasurement<double> &imu_packet_in)
    {
        if (!static_bias_initialization_)
        {
            bias_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Bias initialization is set to false.");
            RCLCPP_INFO(node_->get_logger(), "Bias is initialized using prior as:");
            RCLCPP_INFO(node_->get_logger(), "Gyroscope bias: [%f, %f, %f]", bg0_(0), bg0_(1), bg0_(2));
            RCLCPP_INFO(node_->get_logger(), "Accelerometer bias: [%f, %f, %f]", ba0_(0), ba0_(1), ba0_(2));
            return;
        }

        // 基于IMU方向和静态假设初始化偏置
        if (bias_init_vec_.size() < 250)
        {
            Eigen::Vector3d w, a;
            w << imu_packet_in.angular_velocity.x,
                imu_packet_in.angular_velocity.y,
                imu_packet_in.angular_velocity.z;
            a << imu_packet_in.linear_acceleration.x,
                imu_packet_in.linear_acceleration.y,
                imu_packet_in.linear_acceleration.z;
            Eigen::Quaternion<double> quat(imu_packet_in.orientation.w,
                                           imu_packet_in.orientation.x,
                                           imu_packet_in.orientation.y,
                                           imu_packet_in.orientation.z);
            Eigen::Matrix3d R;
            if (use_imu_ori_est_init_bias_)
            {
                R = quat.toRotationMatrix();
            }
            else
            {
                R = Eigen::Matrix3d::Identity();
            }

            Eigen::Vector3d g;
            g << 0, 0, -9.81;
            a = (R.transpose() * (R * a + g)).eval();
            Eigen::Matrix<double, 6, 1> v;
            v << w(0), w(1), w(2), a(0), a(1), a(2);
            bias_init_vec_.push_back(v); // 存储移除重力的IMU数据
        }
        else
        {
            // 计算存储数据的平均偏置
            Eigen::Matrix<double, 6, 1> avg = Eigen::Matrix<double, 6, 1>::Zero();
            for (size_t i = 0; i < bias_init_vec_.size(); ++i)
            {
                avg = (avg + bias_init_vec_[i]).eval();
            }
            avg = (avg / bias_init_vec_.size()).eval();

            RCLCPP_INFO(node_->get_logger(), "IMU bias initialized to: [%f, %f, %f, %f, %f, %f]",
                        avg(0), avg(1), avg(2), avg(3), avg(4), avg(5));
            bg0_ = avg.head<3>();
            ba0_ = avg.tail<3>();
            bias_initialized_ = true;
        }
    }

    void BodyEstimator::initState(const ImuMeasurement<double> &imu_packet_in,
                                  const JointStateMeasurement &joint_state_packet_in, HuskyState &state)
    {
        // 清除滤波器
        filter_.clear();

        // 初始化状态均值
        Eigen::Quaternion<double> quat(imu_packet_in.orientation.w,
                                       imu_packet_in.orientation.x,
                                       imu_packet_in.orientation.y,
                                       imu_packet_in.orientation.z);
        Eigen::Matrix3d R0;

        if (use_imu_ori_est_init_bias_)
        {
            R0 = quat.toRotationMatrix();
        }
        else
        {
            R0 = Eigen::Matrix3d::Identity();
        }

        Eigen::Vector3d v0_body = joint_state_packet_in.getBodyLinearVelocity();
        Eigen::Vector3d v0 = R0 * v0_body; // 初始速度

        Eigen::Vector3d p0 = {0.0, 0.0, 0.0}; // 初始位置，我们设置IMU坐标系为世界坐标系

        inekf::RobotState initial_state;
        initial_state.setRotation(R0);
        initial_state.setVelocity(v0);
        initial_state.setPosition(p0);
        initial_state.setGyroscopeBias(bg0_);
        initial_state.setAccelerometerBias(ba0_);

        // 初始化状态协方差
        initial_state.setRotationCovariance(0.03 * Eigen::Matrix3d::Identity());
        initial_state.setVelocityCovariance(0.01 * Eigen::Matrix3d::Identity());
        initial_state.setPositionCovariance(0.00001 * Eigen::Matrix3d::Identity());
        initial_state.setGyroscopeBiasCovariance(0.0001 * Eigen::Matrix3d::Identity());
        initial_state.setAccelerometerBiasCovariance(0.0025 * Eigen::Matrix3d::Identity());

        filter_.setState(initial_state);

        RCLCPP_INFO(node_->get_logger(), "Robot's state mean is initialized");
        RCLCPP_INFO(node_->get_logger(), "Robot's state covariance is initialized");
        // 注意：这里需要实现RobotState的流输出操作符，或者分别打印各个状态

        // 设置启用标志
        t_prev_ = imu_packet_in.getTime();
        state.setTime(t_prev_);
        imu_prev_ << imu_packet_in.angular_velocity.x,
            imu_packet_in.angular_velocity.y,
            imu_packet_in.angular_velocity.z,
            imu_packet_in.linear_acceleration.x,
            imu_packet_in.linear_acceleration.y,
            imu_packet_in.linear_acceleration.z;

        enabled_ = true;
    }

    void BodyEstimator::initState(const ImuMeasurement<double> &imu_packet_in,
                                  const VelocityMeasurement &velocity_packet_in, HuskyState &state)
    {
        // 清除滤波器
        filter_.clear();

        // 初始化状态均值
        Eigen::Quaternion<double> quat(imu_packet_in.orientation.w,
                                       imu_packet_in.orientation.x,
                                       imu_packet_in.orientation.y,
                                       imu_packet_in.orientation.z);

        Eigen::Matrix3d R0;
        if (use_imu_ori_est_init_bias_)
        {
            R0 = quat.toRotationMatrix();
        }
        else
        {
            R0 = Eigen::Matrix3d::Identity();
        }

        Eigen::Vector3d v0_body = velocity_packet_in.getLinearVelocity();
        Eigen::Vector3d v0 = R0 * v0_body; // 初始速度

        Eigen::Vector3d p0 = {0.0, 0.0, 0.0}; // 初始位置，我们设置IMU坐标系为世界坐标系

        inekf::RobotState initial_state;
        initial_state.setRotation(R0);
        initial_state.setVelocity(v0);
        initial_state.setPosition(p0);
        initial_state.setGyroscopeBias(bg0_);
        initial_state.setAccelerometerBias(ba0_);

        // 初始化状态协方差
        initial_state.setRotationCovariance(0.03 * Eigen::Matrix3d::Identity());
        initial_state.setVelocityCovariance(0.01 * Eigen::Matrix3d::Identity());
        initial_state.setPositionCovariance(0.00001 * Eigen::Matrix3d::Identity());
        initial_state.setGyroscopeBiasCovariance(0.0001 * Eigen::Matrix3d::Identity());
        initial_state.setAccelerometerBiasCovariance(0.0025 * Eigen::Matrix3d::Identity());

        filter_.setState(initial_state);

        RCLCPP_INFO(node_->get_logger(), "Robot's state mean is initialized");
        RCLCPP_INFO(node_->get_logger(), "Robot's state covariance is initialized");

        // 设置启用标志
        t_prev_ = imu_packet_in.getTime();
        state.setTime(t_prev_);
        imu_prev_ << imu_packet_in.angular_velocity.x,
            imu_packet_in.angular_velocity.y,
            imu_packet_in.angular_velocity.z,
            imu_packet_in.linear_acceleration.x,
            imu_packet_in.linear_acceleration.y,
            imu_packet_in.linear_acceleration.z;
        enabled_ = true;
    }

    inekf::InEKF BodyEstimator::getFilter() const
    {
        return filter_;
    }

    inekf::RobotState BodyEstimator::getState() const
    {
        return filter_.getState();
    }

} // end husky_inekf namespace