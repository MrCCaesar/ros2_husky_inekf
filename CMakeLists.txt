cmake_minimum_required(VERSION 3.8)
project(husky_inekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准为C++14（ROS2推荐）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 优化选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -DEIGEN_NO_DEBUG -march=native")

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")
message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")

# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ROS2依赖列表
set(ROS2_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# 1. 基础库：utils（数学函数）
add_library(utils src/utils/utils.cpp)
ament_target_dependencies(utils ${ROS2_DEPENDENCIES})

# 2. 基础库：measurement（测量基类）
add_library(husky_measurement src/utils/measurement.cpp)
ament_target_dependencies(husky_measurement ${ROS2_DEPENDENCIES})

# 3. InEKF核心库
file(GLOB INEKF_CORE_SOURCES "src/core/*.cpp")
add_library(inekf_core ${INEKF_CORE_SOURCES})
ament_target_dependencies(inekf_core ${ROS2_DEPENDENCIES})
target_link_libraries(inekf_core utils husky_measurement)

# 4. 系统状态库
add_library(husky_state src/system/husky_state.cpp)
ament_target_dependencies(husky_state ${ROS2_DEPENDENCIES})
target_link_libraries(husky_state utils husky_measurement)

# 5. 估计器库
add_library(body_estimator src/estimator/body_estimator.cpp)
ament_target_dependencies(body_estimator ${ROS2_DEPENDENCIES})
target_link_libraries(body_estimator utils husky_measurement inekf_core)

# 6. 通信库
add_library(husky_comms src/communication/husky_comms.cpp)
ament_target_dependencies(husky_comms ${ROS2_DEPENDENCIES})
target_link_libraries(husky_comms husky_measurement)

# 7. 系统库
add_library(husky_system src/system/husky_system.cpp)
ament_target_dependencies(husky_system ${ROS2_DEPENDENCIES})
target_link_libraries(husky_system 
  utils
  husky_measurement
  husky_state
  body_estimator
)

# 8. 姿态发布器库
add_library(pose_publisher_node src/communication/pose_publisher_node.cpp)
ament_target_dependencies(pose_publisher_node ${ROS2_DEPENDENCIES})
target_link_libraries(pose_publisher_node husky_state husky_measurement)

# 主可执行文件
add_executable(husky_estimator src/husky_estimator.cpp)
target_link_libraries(husky_estimator
  husky_comms
  husky_system
  husky_state
  body_estimator
  pose_publisher_node
  inekf_core
  husky_measurement
  utils
  ${Boost_LIBRARIES}
)
ament_target_dependencies(husky_estimator ${ROS2_DEPENDENCIES})

# 其他可执行文件
add_executable(path_publisher_node src/communication/path_publisher_node.cpp)
target_link_libraries(path_publisher_node ${Boost_LIBRARIES})
ament_target_dependencies(path_publisher_node ${ROS2_DEPENDENCIES})

add_executable(pathodom_listener_node src/communication/pathodom_listener_node.cpp)
target_link_libraries(pathodom_listener_node ${Boost_LIBRARIES})
ament_target_dependencies(pathodom_listener_node ${ROS2_DEPENDENCIES})

add_executable(gps_listener_node src/communication/gps_listener_node.cpp)
target_link_libraries(gps_listener_node ${Boost_LIBRARIES})
ament_target_dependencies(gps_listener_node ${ROS2_DEPENDENCIES})

# 安装
install(TARGETS 
  husky_estimator
  path_publisher_node
  pathodom_listener_node
  gps_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(DIRECTORY 
  rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()