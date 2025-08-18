#!/usr/bin/env python3
"""
FRONTIER Launch File
Launches the FRONTIER node with appropriate configurations
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function for launch file"""
    
    # Get package share directory
    frontier_share = get_package_share_directory('frontier')
    
    # Configuration file path
    config_file = os.path.join(
        frontier_share,
        'config',
        'frontier_config.yaml'
    )
    
    # Check if config file exists
    if not os.path.exists(config_file):
        # Fallback to source directory during development
        config_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config',
            'frontier_config.yaml'
        )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    
    # FRONTIER node
    frontier_node = Node(
        package='frontier',
        executable='frontier_node',
        name='frontier',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'use_sim_time': use_sim_time == 'true'}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            # Remap topics if needed (can be configured via launch arguments)
            # ('/frontier/fused_detections', '/detection/fused_objects'),
        ]
    )
    
    # Log information
    log_info = LogInfo(
        msg=f"Starting FRONTIER node with config: {config_file}"
    )
    
    return [log_info, frontier_node]


def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )
    
    # Additional optional arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    
    # Camera configuration arguments
    camera1_topic_arg = DeclareLaunchArgument(
        'camera1_yolo_topic',
        default_value='/camera_1/yolo_detections',
        description='YOLO detection topic for camera 1'
    )
    
    camera2_topic_arg = DeclareLaunchArgument(
        'camera2_yolo_topic',
        default_value='/camera_2/yolo_detections',
        description='YOLO detection topic for camera 2'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_detections_topic',
        default_value='/cone_detection/sorted_cones_time',
        description='LiDAR 3D detection topic'
    )
    
    # Visualization arguments
    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Publish RViz visualization markers'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(log_level_arg)
    ld.add_action(namespace_arg)
    ld.add_action(camera1_topic_arg)
    ld.add_action(camera2_topic_arg)
    ld.add_action(lidar_topic_arg)
    ld.add_action(publish_markers_arg)
    
    # Add opaque function for setup
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld