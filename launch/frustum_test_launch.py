#!/usr/bin/env python3
"""
Frustum Test Launch File
테스트 및 캘리브레이션 검증을 위한 frustum 시각화 전용 launch
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
    camera_id = LaunchConfiguration('camera_id').perform(context)
    near_distance = LaunchConfiguration('near_distance').perform(context)
    far_distance = LaunchConfiguration('far_distance').perform(context)
    use_manual_bbox = LaunchConfiguration('use_manual_bbox').perform(context)
    manual_bbox_center_x = LaunchConfiguration('manual_bbox_center_x').perform(context)
    manual_bbox_center_y = LaunchConfiguration('manual_bbox_center_y').perform(context)
    manual_bbox_width = LaunchConfiguration('manual_bbox_width').perform(context)
    manual_bbox_height = LaunchConfiguration('manual_bbox_height').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    
    # Safety parameters
    max_frustums_per_frame = LaunchConfiguration('max_frustums_per_frame').perform(context)
    min_process_interval = LaunchConfiguration('min_process_interval').perform(context)
    drop_old_messages = LaunchConfiguration('drop_old_messages').perform(context)
    message_age_threshold = LaunchConfiguration('message_age_threshold').perform(context)
    
    # Frustum visualizer node
    frustum_visualizer_node = Node(
        package='frontier',
        executable='frustum_visualizer_node',
        name='frustum_visualizer',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'camera_id': camera_id},
            {'near_distance': float(near_distance)},
            {'far_distance': float(far_distance)},
            {'use_manual_bbox': use_manual_bbox == 'true'},  # Fixed: 'true' means use manual
            {'manual_bbox_center_x': float(manual_bbox_center_x)},
            {'manual_bbox_center_y': float(manual_bbox_center_y)},
            {'manual_bbox_width': float(manual_bbox_width)},
            {'manual_bbox_height': float(manual_bbox_height)},
            {'publish_rate': float(publish_rate)},
            # ROS time
            {'use_sim_time': use_sim_time == 'true'},
            # Safety parameters
            {'max_frustums_per_frame': int(max_frustums_per_frame)},
            {'min_process_interval': float(min_process_interval)},
            {'drop_old_messages': drop_old_messages == 'true'},
            {'message_age_threshold': float(message_age_threshold)}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # Log information
    mode_info = "manual bbox mode" if use_manual_bbox == 'true' else f"YOLO detection mode ({camera_id})"
    log_info = LogInfo(
        msg=f"Starting Frustum Visualizer in {mode_info}\n"
            f"  Config: {config_file}\n"
            f"  Near/Far: {near_distance}m / {far_distance}m\n"
            f"  Publish rate: {publish_rate} Hz"
    )
    
    nodes = [log_info, frustum_visualizer_node]
    
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
    
    # Camera selection
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='camera_1',
        description='Camera ID to use (camera_1 or camera_2)'
    )
    
    # Frustum distance parameters
    near_distance_arg = DeclareLaunchArgument(
        'near_distance',
        default_value='1.0',
        description='Near plane distance in meters'
    )
    
    far_distance_arg = DeclareLaunchArgument(
        'far_distance',
        default_value='30.0',
        description='Far plane distance in meters'
    )
    
    # Manual bbox mode - YOLO detection is default
    use_manual_bbox_arg = DeclareLaunchArgument(
        'use_manual_bbox',
        default_value='false',  # YOLO detection by default
        description='Use manual bounding box instead of YOLO detections',
        choices=['true', 'false']
    )
    
    # Manual bbox parameters
    manual_bbox_center_x_arg = DeclareLaunchArgument(
        'manual_bbox_center_x',
        default_value='320.0',
        description='Manual bbox center X coordinate (pixels)'
    )
    
    manual_bbox_center_y_arg = DeclareLaunchArgument(
        'manual_bbox_center_y',
        default_value='240.0',
        description='Manual bbox center Y coordinate (pixels)'
    )
    
    manual_bbox_width_arg = DeclareLaunchArgument(
        'manual_bbox_width',
        default_value='100.0',
        description='Manual bbox width (pixels)'
    )
    
    manual_bbox_height_arg = DeclareLaunchArgument(
        'manual_bbox_height',
        default_value='100.0',
        description='Manual bbox height (pixels)'
    )
    
    # Publish rate
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz (for manual mode)'
    )
    
    # Safety parameters
    max_frustums_per_frame_arg = DeclareLaunchArgument(
        'max_frustums_per_frame',
        default_value='5',
        description='Maximum number of frustums to process per frame'
    )
    
    min_process_interval_arg = DeclareLaunchArgument(
        'min_process_interval',
        default_value='0.1',
        description='Minimum interval between processing (seconds)'
    )
    
    drop_old_messages_arg = DeclareLaunchArgument(
        'drop_old_messages',
        default_value='true',
        description='Drop messages older than threshold',
        choices=['true', 'false']
    )
    
    message_age_threshold_arg = DeclareLaunchArgument(
        'message_age_threshold',
        default_value='0.5',
        description='Maximum message age in seconds'
    )
    
    # Log level
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level',
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
    ld.add_action(camera_id_arg)
    ld.add_action(near_distance_arg)
    ld.add_action(far_distance_arg)
    ld.add_action(use_manual_bbox_arg)
    ld.add_action(manual_bbox_center_x_arg)
    ld.add_action(manual_bbox_center_y_arg)
    ld.add_action(manual_bbox_width_arg)
    ld.add_action(manual_bbox_height_arg)
    ld.add_action(publish_rate_arg)
    ld.add_action(max_frustums_per_frame_arg)
    ld.add_action(min_process_interval_arg)
    ld.add_action(drop_old_messages_arg)
    ld.add_action(message_age_threshold_arg)
    ld.add_action(log_level_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_rviz_arg)
    
    # Add info message
    ld.add_action(LogInfo(msg="""
    ========================================
    FRONTIER Frustum Visualization Test
    ========================================
    
    This launch file tests frustum generation and visualization.
    
    Usage Examples:
    
    1. YOLO detection mode (default):
       ros2 launch frontier frustum_test_launch.py
       
    2. Change camera:
       ros2 launch frontier frustum_test_launch.py camera_id:=camera_2
    
    3. Manual bbox mode (for calibration testing only):
       ros2 launch frontier frustum_test_launch.py \\
         use_manual_bbox:=true \\
         manual_bbox_center_x:=400 manual_bbox_center_y:=300
    
    4. With RViz:
       ros2 launch frontier frustum_test_launch.py launch_rviz:=true
    
    Topics:
    - Subscribe: /<camera_id>/detections (if use_manual_bbox:=false)
    - Publish: /frustum_visualizer/markers
    
    ========================================
    """))
    
    # Add opaque function for setup
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld