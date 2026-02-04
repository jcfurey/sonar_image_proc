# Quick Start: Adding Sonar Visualization to Bringup

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
- `/oculus/drawn_sonar` - Main rendered sonar image
- `/oculus/drawn_sonar_osd` - With range/bearing overlay
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
