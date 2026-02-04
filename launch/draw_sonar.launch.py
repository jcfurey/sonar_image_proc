#!/usr/bin/env python3
# Simple ROS2 Launch file for drawing sonar images only

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """
    Simple launch file for sonar image visualization.
    
    Launches a single draw_sonar node to visualize sonar images.
    This can be easily included in other launch files.
    """
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the draw_sonar node'
    )
    
    sonar_topic_arg = DeclareLaunchArgument(
        'sonar_topic',
        default_value='/oculus/sonar_image',
        description='Input sonar image topic'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Use component container for better performance'
    )
    
    color_map_arg = DeclareLaunchArgument(
        'color_map',
        default_value='inferno',
        description='Color map for visualization (inferno, hot, jet, etc.)'
    )
    
    log_scale_arg = DeclareLaunchArgument(
        'log_scale',
        default_value='false',
        description='Use logarithmic scale for intensity'
    )
    
    publish_histogram_arg = DeclareLaunchArgument(
        'publish_histogram',
        default_value='false',
        description='Publish histogram data'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='0.0',
        description='Maximum range to display (0.0 for auto)'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    sonar_topic = LaunchConfiguration('sonar_topic')
    use_composition = LaunchConfiguration('use_composition')
    color_map = LaunchConfiguration('color_map')
    log_scale = LaunchConfiguration('log_scale')
    publish_histogram = LaunchConfiguration('publish_histogram')
    max_range = LaunchConfiguration('max_range')
    
    # Define common parameters
    draw_sonar_params = {
        'publish_histogram': publish_histogram,
        'color_map': color_map,
        'log_scale': log_scale,
        'max_range': max_range,
        'publish_old': False,
        'publish_timing': True,
        'range_spacing': 10.0,
        'bearing_spacing': 10.0,
        'line_alpha': 0.5,
        'line_thickness': 1,
        'min_db': -80.0,
        'max_db': 0.0,
    }
    
    # Component version
    draw_sonar_component = ComposableNode(
        package='sonar_image_proc',
        plugin='draw_sonar::DrawSonarComponent',
        name='draw_sonar',
        namespace=namespace,
        parameters=[draw_sonar_params],
        remappings=[
            ('sonar_image', sonar_topic),
        ],
    )
    
    component_container = ComposableNodeContainer(
        name='draw_sonar_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[draw_sonar_component],
        output='screen',
        condition=IfCondition(use_composition)
    )
    
    # Standalone node version
    draw_sonar_node = Node(
        package='sonar_image_proc',
        executable='draw_sonar_node',
        name='draw_sonar',
        namespace=namespace,
        parameters=[draw_sonar_params],
        remappings=[
            ('sonar_image', sonar_topic),
        ],
        output='screen',
        condition=UnlessCondition(use_composition)
    )
    
    return LaunchDescription([
        namespace_arg,
        sonar_topic_arg,
        use_composition_arg,
        color_map_arg,
        log_scale_arg,
        publish_histogram_arg,
        max_range_arg,
        component_container,
        draw_sonar_node,
    ])
