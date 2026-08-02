#!/usr/bin/env python3
# Simple ROS2 Launch file for drawing sonar images only

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch sonar image visualization.

    Launches a single draw_sonar node to visualize sonar images.
    This can be easily included in other launch files.
    """
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the draw_sonar node'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=EnvironmentVariable('use_sim_time', default_value='False'),
        description='Use the /clock topic for ROS time',
    )

    sonar_topic_arg = DeclareLaunchArgument(
        'sonar_topic',
        default_value='/oculus/sonar_image',
        description='Input sonar image topic'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description=(
            'Path to a YAML parameter file. If provided, inline parameters '
            'are ignored.'
        )
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

    pixels_per_meter_arg = DeclareLaunchArgument(
        'pixels_per_meter',
        default_value='0.0',
        description=(
            'Output image scale in pixels per meter. 0 (default) scales to '
            'the ping\'s native range resolution (one pixel per range bin), '
            'matching the node default; a fixed value overrides it.'
        )
    )

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sonar_topic = LaunchConfiguration('sonar_topic')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    color_map = LaunchConfiguration('color_map')
    log_scale = LaunchConfiguration('log_scale')
    publish_histogram = LaunchConfiguration('publish_histogram')
    max_range = LaunchConfiguration('max_range')
    pixels_per_meter = LaunchConfiguration('pixels_per_meter')

    # Parameter-file selection must happen with a live LaunchContext. Resolving
    # a LaunchConfiguration while constructing the description raises before
    # ROS can launch anything.
    def launch_setup(context):
        params_file_path = params_file.perform(context)
        if params_file_path:
            draw_sonar_params = [params_file_path]
        else:
            draw_sonar_params = [{
                'publish_histogram': publish_histogram,
                'color_map': color_map,
                'log_scale': log_scale,
                'max_range': max_range,
                'pixels_per_meter': pixels_per_meter,
                'publish_old': False,
                'publish_timing': True,
                'range_spacing': 10.0,
                'bearing_spacing': 10.0,
                'line_alpha': 0.5,
                'line_thickness': 1,
                'min_db': 0.0,  # auto: the sample type's LSB floor; -80 washed out 16-bit data
                'max_db': 0.0,
            }]
        draw_sonar_params.append({'use_sim_time': use_sim_time})

        draw_sonar_component = ComposableNode(
            package='sonar_image_proc',
            plugin='draw_sonar::DrawSonarComponent',
            name='draw_sonar',
            namespace=namespace,
            parameters=draw_sonar_params,
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

        draw_sonar_node = Node(
            package='sonar_image_proc',
            executable='draw_sonar_node',
            name='draw_sonar',
            namespace=namespace,
            parameters=draw_sonar_params,
            remappings=[
                ('sonar_image', sonar_topic),
            ],
            output='screen',
            condition=UnlessCondition(use_composition)
        )

        return [component_container, draw_sonar_node]

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        sonar_topic_arg,
        params_file_arg,
        use_composition_arg,
        color_map_arg,
        log_scale_arg,
        publish_histogram_arg,
        max_range_arg,
        pixels_per_meter_arg,
        OpaqueFunction(function=launch_setup),
    ])
