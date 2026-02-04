#!/usr/bin/env python3
# ROS2 Launch file for sonar image processing pipeline

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    """
    Launch file for sonar image processing.
    
    Creates two processing pipelines:
    1. Raw sonar processing (in /oculus namespace)
    2. Postprocessed sonar processing (in /postprocess namespace)
    
    Both pipelines include:
    - draw_sonar: Draws sonar images to OpenCV format
    - Optional histogram generation
    """
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for the nodes'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Path to YAML params file (e.g., oculus/common.yaml). If provided, other parameters are ignored.'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Use component container for nodes (recommended for performance)'
    )
    
    publish_histogram_arg = DeclareLaunchArgument(
        'publish_histogram',
        default_value='false',
        description='Publish histogram data from draw_sonar nodes'
    )
    
    color_map_arg = DeclareLaunchArgument(
        'color_map',
        default_value='inferno',
        description='Color map to use for sonar visualization (inferno, hot, jet, etc.)'
    )
    
    log_scale_arg = DeclareLaunchArgument(
        'log_scale',
        default_value='false',
        description='Use logarithmic scale for intensity'
    )
    
    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='1.0',
        description='Gain multiplier for postprocessor (1.0 = no change)'
    )
    
    gamma_arg = DeclareLaunchArgument(
        'gamma',
        default_value='0.0',
        description='Gamma correction for postprocessor (0.0 = no correction)'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    publish_histogram = LaunchConfiguration('publish_histogram')
    color_map = LaunchConfiguration('color_map')
    log_scale = LaunchConfiguration('log_scale')
    gain = LaunchConfiguration('gain')
    gamma = LaunchConfiguration('gamma')
    
    # Helper to conditionally load params from file or use inline
    def get_draw_sonar_params():
        params_file_str = params_file.perform(None) if hasattr(params_file, 'perform') else ''
        if params_file_str and os.path.exists(params_file_str):
            # Load from file
            return [params_file_str]
        else:
            # Use inline parameters
            return [{
                'publish_histogram': publish_histogram,
                'color_map': color_map,
                'log_scale': log_scale,
                'max_range': 0.0,
                'publish_old': False,
                'publish_timing': True,
                'range_spacing': 10.0,
                'bearing_spacing': 10.0,
                'line_alpha': 0.5,
                'line_thickness': 1,
                'min_db': -80.0,
                'max_db': 0.0,
            }]
    
    # Raw sonar processing components
    raw_draw_sonar = ComposableNode(
        package='sonar_image_proc',
        plugin='draw_sonar::DrawSonarComponent',
        name='draw_sonar',
        namespace='oculus',
        parameters=get_draw_sonar_params(),
        remappings=[
            ('sonar_image', '/oculus/sonar_image'),
        ],
    )
    
    # Postprocessing pipeline components
    sonar_postprocessor = ComposableNode(
        package='sonar_image_proc',
        plugin='sonar_postprocessor::SonarPostprocessorComponent',
        name='sonar_postprocessor',
        namespace='postprocess',
        parameters=[{
            'gain': gain,
            'gamma': gamma,
        }],
        remappings=[
            ('sonar_image', '/oculus/sonar_image'),
            ('sonar_image_postproc', '/postprocess/sonar_image'),
        ],
    )
    
    postprocess_draw_sonar = ComposableNode(
        package='sonar_image_proc',
        plugin='draw_sonar::DrawSonarComponent',
        name='draw_sonar',
        namespace='postprocess',
        parameters=get_draw_sonar_params(),
        remappings=[
            ('sonar_image', '/postprocess/sonar_image'),
        ],
    )
    
    # Component container approach (recommended)
    component_container = ComposableNodeContainer(
        name='sonar_image_proc_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            raw_draw_sonar,
            sonar_postprocessor,
            postprocess_draw_sonar,
        ],
        output='screen',
        condition=IfCondition(use_composition)
    )
    
    # Standalone node approach (fallback)
    raw_draw_sonar_node = Node(
        package='sonar_image_proc',
        executable='draw_sonar_node',
        name='draw_sonar',
        namespace='oculus',
        parameters=get_draw_sonar_params(),
        remappings=[
            ('sonar_image', '/oculus/sonar_image'),
        ],
        output='screen',
        condition=UnlessCondition(use_composition)
    )
    
    postprocessor_node = Node(
        package='sonar_image_proc',
        executable='sonar_postprocessor_node',
        name='sonar_postprocessor',
        namespace='postprocess',
        parameters=[{
            'gain': gain,
            'gamma': gamma,
        }],
        remappings=[
            ('sonar_image', '/oculus/sonar_image'),
            ('sonar_image_postproc', '/postprocess/sonar_image'),
        ],
        output='screen',
        condition=UnlessCondition(use_composition)
    )
    
    postprocess_draw_sonar_node = Node(
        package='sonar_image_proc',
        executable='draw_sonar_node',
        name='draw_sonar',
        namespace='postprocess',
        parameters=get_draw_sonar_params(),
        remappings=[
            ('sonar_image', '/postprocess/sonar_image'),
        ],
        output='screen',
        condition=UnlessCondition(use_composition)
    )
    
    # Create and return launch description
    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        use_composition_arg,
        publish_histogram_arg,
        color_map_arg,
        log_scale_arg,
        gain_arg,
        gamma_arg,
        component_container,
        raw_draw_sonar_node,
        postprocessor_node,
        postprocess_draw_sonar_node,
    ])
