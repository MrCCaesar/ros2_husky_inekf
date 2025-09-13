#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    husky_namespace_arg = DeclareLaunchArgument(
        'husky_namespace',
        default_value='husky',
        description='Namespace for the husky robot'
    )
    
    # Get package directory
    pkg_husky_inekf = get_package_share_directory('husky_inekf')
    
    # Parameter files paths
    settings_config = os.path.join(pkg_husky_inekf, 'config', 'settings.yaml')
    noise_config = os.path.join(pkg_husky_inekf, 'config', 'noise.yaml') 
    prior_config = os.path.join(pkg_husky_inekf, 'config', 'prior.yaml')
    
    # Main husky InEKF estimator node
    husky_inekf_node = Node(
        package='husky_inekf',
        executable='husky_estimator',
        name='husky_estimator',
        output='both',
        parameters=[
            {'use_sim_time': True},
            settings_config,
            noise_config,
            prior_config
        ]
    )
    
    # InEKF path publisher node
    path_publisher_node = Node(
        package='husky_inekf',
        executable='path_publisher_node',
        name='InEKF_path_publisher',
        output='both',
        parameters=[
            settings_config
            # Commented out file_name parameter as in original
            # {'file_name': '/media/curly_ssd_justin/code/minicheetah-perception/catkin_ws/src/cheetah_inekf_lcm/data/08292020_trail1_gt_2.txt'}
        ]
    )
    
    # Path-odom listener node
    pathodom_listener_node = Node(
        package='husky_inekf',
        executable='pathodom_listener_node',
        name='InEKF_pathodom_litsener',
        output='both',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    # GPS listener node
    gps_listener_node = Node(
        package='husky_inekf',
        executable='gps_listener_node', 
        name='InEKF_gps_litsener',
        output='both',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    # Static transform publisher (map -> odom)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map',
        arguments=[
            '0', '0', '0',      # translation x, y, z
            '0', '0', '0',      # rotation x, y, z (euler angles)
            'map',              # parent frame
            'odom'              # child frame
        ],
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    rviz_config = os.path.join(pkg_husky_inekf, 'rviz', 'default.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],  # 使用 -d 参数指定配置文件
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        # Launch arguments
        husky_namespace_arg,
        
        # Nodes
        husky_inekf_node,
        path_publisher_node,
        pathodom_listener_node,
        gps_listener_node,
        static_tf_node,
        
        # Commented nodes
        rviz_node,
    ])