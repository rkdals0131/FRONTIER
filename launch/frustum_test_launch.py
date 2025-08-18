#!/usr/bin/env python3
"""
Frustum Test Launch File
테스트 및 디버깅을 위해 FRONTIER 메인 노드를 실행하고 RViz 시각화를 제공
실제 파이프라인을 그대로 사용하면서 디버그 옵션 활성화
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
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
    
    # Check if config file exists (fallback to source during development)
    if not os.path.exists(config_file):
        config_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config',
            'frontier_config.yaml'
        )
    
    # Get launch configurations
    log_level = LaunchConfiguration('log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    
    # FRONTIER main node - 실제 파이프라인 사용
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
            # 필요시 토픽 리매핑 가능
        ]
    )
    
    # Log information
    log_info = LogInfo(
        msg=f"Starting FRONTIER node in test/debug mode\n"
            f"  Config: {config_file}\n"
            f"  Log level: {log_level}\n"
            f"  Visualization topic: /frontier/visualization"
    )
    
    nodes = [log_info, frontier_node]
    
    # Optionally launch RViz
    if launch_rviz == 'true':
        # Check for RViz config file
        rviz_config_file = os.path.join(frontier_share, 'config', 'frustum_test.rviz')
        
        # If no custom config exists, use basic config
        if not os.path.exists(rviz_config_file):
            rviz_config_file = None
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file] if rviz_config_file else [],
        )
        
        nodes.append(rviz_node)
        
        # Add log message for RViz
        rviz_log = LogInfo(msg="Launching RViz2 for visualization")
        nodes.append(rviz_log)
    
    return nodes


def generate_launch_description():
    """Generate launch description"""
    
    # Log level
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (use debug for detailed visualization info)',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )
    
    # ROS2 sim time parameter
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (for rosbag replay or simulation)',
        choices=['true', 'false']
    )
    
    # RViz launch option
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz2 for visualization',
        choices=['true', 'false']
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(log_level_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_rviz_arg)
    
    # Add info message
    ld.add_action(LogInfo(msg="""
    ========================================
    FRONTIER Test/Debug Launch
    ========================================
    
    실제 FRONTIER 파이프라인을 실행하고 RViz에서 시각화합니다.
    camera_1과 camera_2 모두 동시에 처리됩니다.
    
    Usage Examples:
    
    1. 기본 실행 (RViz 포함):
       ros2 launch frontier frustum_test_launch.py
       
    2. 디버그 모드 (상세 로그):
       ros2 launch frontier frustum_test_launch.py log_level:=debug
    
    3. RViz 없이 실행:
       ros2 launch frontier frustum_test_launch.py launch_rviz:=false
    
    Input Topics:
    - /camera_1/detections - Camera 1 YOLO detections
    - /camera_2/detections - Camera 2 YOLO detections  
    - /cone/lidar/box - LiDAR 3D detections
    
    Output Topics:
    - /frontier/fused_detections - Matched 3D objects
    - /frontier/visualization - RViz visualization markers
    
    시각화 색상:
    - Camera 1: 노란색 (Yellow)
    - Camera 2: 청록색 (Cyan)
    
    ========================================
    """))
    
    # Add opaque function for setup
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld