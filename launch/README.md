# Sonar Image Processing Launch Files

This directory contains ROS2 launch files for the sonar_image_proc package.

## Available Launch Files

### 1. draw_sonar.launch.py

Simple launch file for visualizing sonar images with a single draw_sonar node.

**Usage:**
```bash
ros2 launch sonar_image_proc draw_sonar.launch.py
```

**Arguments:**
- `namespace` (default: ''): Namespace for the draw_sonar node
- `sonar_topic` (default: '/oculus/sonar_image'): Input sonar image topic
- `use_composition` (default: 'true'): Use component container for better performance
- `color_map` (default: 'inferno'): Color map for visualization (inferno, hot, jet, etc.)
- `log_scale` (default: 'false'): Use logarithmic scale for intensity
- `publish_histogram` (default: 'false'): Publish histogram data
- `max_range` (default: '0.0'): Maximum range to display (0.0 for auto)

**Example:**
```bash
# Use with custom topic and color map
ros2 launch sonar_image_proc draw_sonar.launch.py \
    sonar_topic:=/my_sonar/image \
    color_map:=hot \
    namespace:=sonar_viz
```

**Including in other launch files (Python):**
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sonar_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sonar_image_proc'),
                'launch',
                'draw_sonar.launch.py'
            )
        ]),
        launch_arguments={
            'namespace': 'sensors',
            'sonar_topic': '/oculus/sonar_image',
            'color_map': 'inferno',
        }.items()
    )
    
    return LaunchDescription([sonar_viz_launch])
```

**Including in XML launch files:**
```xml
<launch>
  <include file="$(find-pkg-share sonar_image_proc)/launch/draw_sonar.launch.py">
    <arg name="namespace" value="sensors"/>
    <arg name="sonar_topic" value="/oculus/sonar_image"/>
    <arg name="color_map" value="inferno"/>
  </include>
</launch>
```

### 2. sonar_postproc.launch.py

Full processing pipeline with both raw and post-processed sonar visualization.

**Usage:**
```bash
ros2 launch sonar_image_proc sonar_postproc.launch.py
```

**Arguments:**
- `namespace` (default: ''): Top-level namespace for the nodes
- `use_composition` (default: 'true'): Use component container for nodes
- `publish_histogram` (default: 'false'): Publish histogram data from draw_sonar nodes
- `color_map` (default: 'inferno'): Color map to use for sonar visualization
- `log_scale` (default: 'false'): Use logarithmic scale for intensity

**What it launches:**

This creates two processing pipelines:

1. **Raw sonar pipeline** (namespace: `/oculus`)
   - `draw_sonar`: Visualizes raw sonar images
   - Input: `/oculus/sonar_image`
   - Outputs: `/oculus/drawn_sonar`, `/oculus/drawn_sonar_clean`, `/oculus/drawn_sonar_osd`, `/oculus/drawn_sonar_rect`

2. **Postprocessed pipeline** (namespace: `/postprocess`)
   - `sonar_postprocessor`: Applies gain/gamma corrections
   - `draw_sonar`: Visualizes post-processed sonar images
   - Input: `/oculus/sonar_image`
   - Intermediate: `/postprocess/sonar_image`
   - Outputs: `/postprocess/drawn_sonar`, `/postprocess/drawn_sonar_clean`, `/postprocess/drawn_sonar_osd`, `/postprocess/drawn_sonar_rect`

**Example:**
```bash
# Launch with histogram publishing and hot color map
ros2 launch sonar_image_proc sonar_postproc.launch.py \
    publish_histogram:=true \
    color_map:=hot
```

## Published Topics

### draw_sonar node outputs:
- `drawn_sonar` (sensor_msgs/Image): Annotated operator fan with range and bearing labels
- `drawn_sonar_clean` (sensor_msgs/Image): Annotation-free fan for machine vision
- `drawn_sonar_osd` (sensor_msgs/Image): Compatibility alias of `drawn_sonar`
- `drawn_sonar_rect` (sensor_msgs/Image): Rectified sonar image
- `sonar_image_proc_timing` (std_msgs/String): Processing timing information
- `histogram` (std_msgs/UInt32MultiArray): Histogram data (if enabled)

### sonar_postprocessor node:
- Subscribes to: `sonar_image` (marine_acoustic_msgs/ProjectedSonarImage)
- Publishes to: `sonar_image_postproc` (marine_acoustic_msgs/ProjectedSonarImage)

## Parameters

### draw_sonar parameters:
- `color_map`: Color map name (inferno, hot, jet, etc.)
- `log_scale`: Use logarithmic intensity scale
- `max_range`: Maximum range to display (0.0 = auto)
- `publish_histogram`: Enable histogram publishing
- `publish_timing`: Enable timing info publishing
- `range_spacing`: Spacing between range circles (meters)
- `bearing_spacing`: Spacing between bearing lines (degrees)
- `line_alpha`: Opacity of overlay lines (0.0-1.0)
- `line_thickness`: Thickness of overlay lines (pixels)
- `min_db`: Minimum intensity in dB (for log scale)
- `max_db`: Maximum intensity in dB (for log scale)

### sonar_postprocessor parameters:
- `gain`: Intensity gain multiplier (default: 1.0)
- `gamma`: Gamma correction value (default: 0.0)

## Integration with Bringup Launch Files

To include sonar visualization in your platform bringup:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Your existing bringup nodes...
    
    # Add sonar visualization
    sonar_viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sonar_image_proc'),
                'launch',
                'draw_sonar.launch.py'
            )
        ])
    )
    
    return LaunchDescription([
        # ... your other nodes ...
        sonar_viz,
    ])
```

## Component vs Standalone Nodes

The launch files support both component-based (composable nodes in a container) and standalone node execution:

- **Component mode** (`use_composition:=true`, default): 
  - Better performance (shared memory, no serialization overhead)
  - All nodes run in a single process
  - Recommended for production use

- **Standalone mode** (`use_composition:=false`):
  - Each node runs in its own process
  - Easier debugging with gdb/valgrind
  - Better isolation if a node crashes
