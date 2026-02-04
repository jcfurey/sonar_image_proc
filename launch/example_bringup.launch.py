#!/usr/bin/env python3
"""
Example: How to include sonar_image_proc in a bringup launch file

This demonstrates how to integrate sonar visualization into your platform's
bringup launch configuration.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Example bringup launch file that includes sonar visualization.
    """
    
    # Include sonar visualization (simple version)
    sonar_viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sonar_image_proc'),
                'launch',
                'draw_sonar.launch.py'
            )
        ]),
        launch_arguments={
            'namespace': '',
            'sonar_topic': '/oculus/sonar_image',
            'color_map': 'inferno',
            'log_scale': 'false',
        }.items()
    )
    
    # OR include full processing pipeline (raw + postprocessed)
    # sonar_full_pipeline = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('sonar_image_proc'),
    #             'launch',
    #             'sonar_postproc.launch.py'
    #         )
    #     ]),
    #     launch_arguments={
    #         'namespace': '',
    #         'color_map': 'inferno',
    #         'publish_histogram': 'false',
    #     }.items()
    # )
    
    return LaunchDescription([
        # Add your other bringup nodes here...
        # robot_state_publisher,
        # controller_manager,
        # sensor_drivers,
        
        # Include sonar visualization
        sonar_viz,
        
        # Or use the full pipeline:
        # sonar_full_pipeline,
    ])
