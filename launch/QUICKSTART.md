# Quick Start: Adding Sonar Visualization to Bringup

## Important: Parameters Note

The `oculus/common.yaml` file contains parameters for the **oculus_sonar_driver** (hardware driver), 
not for sonar_image_proc. 

For sonar_image_proc visualization parameters, use the example in `params/draw_sonar_default.yaml` or pass parameters inline.

## TL;DR - Copy & Paste for Bringup Launch Files

Add this to your bringup launch file (Python):

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Inside your generate_launch_description():
sonar_viz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory('sonar_image_proc'),
            'launch',
            'draw_sonar.launch.py'
        )
    ])
)

# Add 'sonar_viz' to your LaunchDescription list
```

## For XML Launch Files

```xml
<launch>
  <!-- Your existing nodes... -->
  
  <include file="$(find-pkg-share sonar_image_proc)/launch/draw_sonar.launch.py">
    <arg name="sonar_topic" value="/oculus/sonar_image"/>
  </include>
</launch>
```

## Testing the Launch Files

```bash
# Test basic visualization
ros2 launch sonar_image_proc draw_sonar.launch.py

# Test with params file
ros2 launch sonar_image_proc draw_sonar.launch.py \
    params_file:=$(ros2 pkg prefix sonar_image_proc)/share/sonar_image_proc/params/draw_sonar_default.yaml

# Test full pipeline (raw + postprocessed)
ros2 launch sonar_image_proc sonar_postproc.launch.py

# With custom parameters
ros2 launch sonar_image_proc draw_sonar.launch.py \
    sonar_topic:=/my/sonar/topic \
    color_map:=hot \
    log_scale:=true
```

## Key Topics Published

After launching, you'll get:
- `/oculus/drawn_sonar` - Annotated operator image with range/bearing labels
- `/oculus/drawn_sonar_clean` - Annotation-free machine-vision image
- `/oculus/drawn_sonar_osd` - Compatibility alias of `drawn_sonar`
- `/oculus/drawn_sonar_rect` - Rectified view

View with:
```bash
ros2 run rqt_image_view rqt_image_view
```

## Files Created

- **draw_sonar.launch.py** - Simple single-node launcher (use this for bringup)
- **sonar_postproc.launch.py** - Full pipeline with postprocessing
- **example_bringup.launch.py** - Example integration
- **README.md** - Detailed documentation

## Need Help?

See [launch/README.md](./README.md) for complete documentation.
