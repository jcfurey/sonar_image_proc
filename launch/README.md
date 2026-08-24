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
   - Outputs: `/oculus/drawn_sonar`, `/oculus/drawn_sonar_clean`, `/oculus/drawn_sonar_osd`, `/oculus/drawn_sonar_polar`, `/oculus/drawn_sonar_rectified`, `/oculus/drawn_sonar_floor_projected`, `/oculus/fan_info`, `/oculus/rectified_info`, `/oculus/floor_projected_camera_info`

2. **Postprocessed pipeline** (namespace: `/postprocess`)
   - `sonar_postprocessor`: Applies gain/gamma corrections
   - `draw_sonar`: Visualizes post-processed sonar images
   - Input: `/oculus/sonar_image`
   - Intermediate: `/postprocess/sonar_image`
   - Outputs: `/postprocess/drawn_sonar`, `/postprocess/drawn_sonar_clean`, `/postprocess/drawn_sonar_osd`, `/postprocess/drawn_sonar_polar`, `/postprocess/drawn_sonar_rectified`, `/postprocess/drawn_sonar_floor_projected`, `/postprocess/fan_info`, `/postprocess/rectified_info`, `/postprocess/floor_projected_camera_info`

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
- `drawn_sonar_polar` (sensor_msgs/Image): Rotated range×bearing inspection image; not camera-rectified
- `drawn_sonar_rectified` (sensor_msgs/Image): Rectilinear, forward-facing range×bearing image (far up, near down); native-height 16:9 by default
- `rectified_info` (sonar_image_proc/RectifiedImageInfo): Exact pixel↔range/bearing mapping for `drawn_sonar_rectified`
- `drawn_sonar_floor_projected` (sensor_msgs/Image): True virtual-pinhole view obtained from the stamped floor plane and live head TF; invalid/out-of-aperture pixels are black
- `floor_projected_camera_info` (sensor_msgs/CameraInfo): Ideal pinhole geometry for `drawn_sonar_floor_projected`
- `fan_info` (sonar_image_proc/FanImageInfo): Per-ping orthographic fan geometry
- `drawn_sonar_rect` / `camera_info`: Deprecated migration aliases
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
- `rectified_width`: Rectified output width (`0` derives it from height and aspect ratio)
- `rectified_height`: Rectified output height (`0` retains native radial resolution)
- `rectified_aspect_ratio`: Width/height used when `rectified_width` is `0` (default `16/9`)
- `floor_projection_sensor_frame`: Sonar projection frame used by the ping geometry (empty uses the ping header)
- `floor_projection_optical_frame`: Optical output frame (empty derives it from a `*/projection_frame` ping frame)
- `floor_projection_surface_frame`: TF frame whose XY plane is projected (default `sea_floor_estimate`)
- `floor_projection_tf_timeout`: Maximum stamped TF lookup wait in seconds

The floor-projected `CameraInfo` spans the ping's bearing limits horizontally
and encloses its transmitted elevation aperture at every bearing. `fx` and
`fy` are therefore usually different; this keeps the narrow physical elevation
aperture framed in the rectangular output instead of inventing a much wider
vertical FOV. The true spherical-wedge aperture has a curved boundary in a
pinhole raster, so out-of-footprint pixels remain black.
- `publish_histogram`: Enable histogram publishing
- `publish_timing`: Enable timing info publishing
- `publish_legacy_camera_info`: Publish deprecated non-pinhole `camera_info`
- `publish_legacy_rect_topic`: Alias `drawn_sonar_polar` as `drawn_sonar_rect`
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
