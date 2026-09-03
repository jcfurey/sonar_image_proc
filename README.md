# sonar_image_proc

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Code to draw data from forward-looking imaging sonars.

If built for ROS, it will build a node/nodelet
[draw_sonar](https://github.com/apl-ocean-engineering/libdraw_sonar/tree/master/src_ros)
which subscribes to an
[marine_acoustic_msgs/ProjectedSonarImage](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs/msg/ProjectedSonarImage.msg)
and publishes a
[sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html).

The core library contains no ROS dependencies, and can be linked into non-ROS applications.

# ROS Interfaces (draw_sonar_node)

`rosrun sonar_image_proc draw_sonar_node`

## Subscribers

Subscribes to the topic `sonar_image` of type [marine_acoustic_msgs/ProjectedSonarImage](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs/msg/ProjectedSonarImage.msg).


## Publishers

By default publishes Cartesian fan images, a polar inspection image, a
forward-facing range-bearing rectangle, and stamped geometry:

* `drawn_sonar` is the operator image: a Cartesian fan with range rings and
bearing rays. Its meter labels sit outside the low-bearing edge beside their
ring markers, and degree labels sit outside the outer ring on their respective
rays, so neither obscures acoustic returns. It has a black display border for
those labels; do not use its pixel dimensions as fan geometry. The color map
used to convert the sonar intensity to RGB is set in code.

* `drawn_sonar_clean` contains the same Cartesian fan without annotations. Use
this topic for optical flow, feature tracking, recording for machine learning,
and other pixel-processing consumers.

![](drawn_sonar.png)

* `drawn_sonar_polar` is the source intensity raster in range×bearing space,
rotated for display so zero range is at the bottom. It is not a rectified camera
image. Since the source data is azimuth-major, before the display rotation:

 * Image width is the number of range bins in the data, with the minimum range
   on the left side and maximum range on the right side.

 * Image height is the number of azimuth bins in the data, with the lowest
   azimuth (typically the most negative) at the top, and most positive at the
   bottom.

![](drawn_sonar_rect.png)

* `drawn_sonar_rectified` is the forward-facing rectangular range×bearing
product. It is generated directly from the source raster, before
the Cartesian fan remap, so it never tries to invert an already-resampled fan.
Far range is at the top, near range at the bottom, and horizontal pixels use a
rectilinear `u = fx * tan(bearing) + cx` projection. By default it retains the
native number of range rows and derives a 16:9 width; fixed dimensions and the
aspect ratio are configurable. `rectified_info`
(`sonar_image_proc/RectifiedImageInfo`) carries the exact inverse mapping from
pixels to range and bearing.

  This is camera-*style* orientation, not an optical camera model: the vertical
  coordinate is measured range because a 2D imaging sonar does not measure
  elevation. Consequently it must not be advertised as `CameraInfo` or overlaid
  pixel-for-pixel on a camera frame without a separate 3D/elevation policy.

* `drawn_sonar_floor_projected` supplies a floor-specific elevation policy. It
is an ideal virtual pinhole image in the configured sonar optical frame;
`floor_projection_sensor_frame` identifies the sonar projection frame in which
the ping's range/bearing convention is expressed. The plane standoff is fitted
from the coherent floor-return onset in that same ping. The detector requires
the return to remain bright through the range band predicted by the measured
transmit aperture, so a thin rail, wall edge, or electronic range ring cannot
win merely by making a sharp line. No DVL altitude or DVL surface frame enters
this product.

  `floor_projection_reference_frame` contributes orientation only: its +z axis
is transformed through the live pivot-head TF into the sonar projection frame.
This resolves the 2-D sonar's otherwise ambiguous elevation branch, including
the cases where floor returns belong above rather than below boresight. For
every output pixel, the node intersects its camera ray with the detected plane,
checks that the inferred elevation is inside the transmitted aperture, and
samples the original processed raster at the resulting physical range and
bearing. Pixels whose rays cannot meet the floor through that aperture remain
black. If the aperture points away from the floor or no coherent return is
detected, the node publishes no floor image instead of fabricating one.

  The horizontal intrinsics span the reported bearing limits and the vertical
intrinsics enclose the ping's measured transmit/elevation aperture at every
bearing. Since an Oculus fan is a wide spherical wedge rather than a rectangular
camera pyramid, its aperture boundary is curved in a pinhole image and some
pixels outside that footprint remain black. `fx` and `fy` are normally
different in the 16:9 raster. Treating them as equal would invent a roughly
100-degree vertical FOV for a 20-degree head and squeeze the usable surface into
a thin strip.

  `floor_projected_camera_info` is valid `sensor_msgs/CameraInfo` for this
ideal virtual camera. The topic name is deliberately surface-specific: the
projection assumes the return lies on the detected floor plane and must not be
treated as recovered obstacle relief. A multi-view elevation estimator can
supply a more general surface model later without changing the pinhole
projection itself.

* `fan_info` (`sonar_image_proc/FanImageInfo`) carries the exact per-ping
orthographic geometry of `drawn_sonar_clean`: dimensions, fan apex, pixels per
metre, range limits, and bearing limits. Consumers should pair it with the
clean image by header stamp; `drawn_sonar` adds an OSD-only border.

* `drawn_sonar_rect` and `camera_info` are deprecated compatibility outputs.
The former aliases `drawn_sonar_polar`; the latter historically stored fan
scale in pinhole-camera fields even though an orthographic fan is not a camera
model. Disable them with `publish_legacy_rect_topic` and
`publish_legacy_camera_info` after all consumers migrate.

* `drawn_sonar_osd` is a compatibility alias of annotated `drawn_sonar` for
existing dashboards.

If the param `publish_timing` is `true`, the node will track the elapsed time to
draw each sonar image and publish that information to the topic `sonar_image_proc_timing`
as a [std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html)
containing a JSON dict.

## Params

If `max_range` is set to a non-zero value, images will be clipped/dropped to that max range (or the actual sonar range, whichever is smaller).

If `publish_timing` is `true` the node will publish performance information as a
JSON [string](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html)
to the topic `sonar_image_proc_timing`.  Defaults to `true`

If `publish_histogram` is `true` the node will publish a "raw" histogram information as a `UInt32MultiArray` to the topic `histogram`.   It contains a vector of unsigned ints giving the count for each intensity value -- so for 8 bit data the vector will be 256 elements in length, and for 16-bit data it will be 65536 elements in length.

Do not run ROS `image_proc` rectification on these sonar outputs. The Cartesian
fan is orthographic, `drawn_sonar_rectified` has bearing/range axes, and
`drawn_sonar_floor_projected` is already an ideal pinhole view with no lens
distortion.

# bag2sonar

The program `bag2sonar` reads in a bagfile containing a `ProjectedSonarImage` topic, draws the sonar image and writes those images to *new* bagfile in a `Image` topic.

Usage:

```
$ rosrun sonar_image_proc bag2sonar
Usage:

   bag2sonar [options]  <input file(s)>

Draw sonar from a bagfile:
  -h [ --help ]                         Display this help message
  -l [ --logscale ]                     Do logscale
  --min-db arg (=0)                     Min db
  --max-db arg (=0)                     Max db
  --osd                                 If set, include the on-screen display
                                        in output
  -o [ --output-bag ] arg               Name of output bagfile
  -t [ --output-topic ] arg (=/drawn_sonar)
                                        Topic for images in output bagfile
```

Note that `bag2sonar` is not a conventional ROS node, it is intended to run as a standalone commandline program.  It uses `ros_storage` to read the input bagfile sequentially, rather than subscribing to a topic.

# histogram_drawer

`python/histogram_drawer` is a Python script which subscribes to the `histogram` topic and uses numpy+Matplotlib to bin the data (into a fixed set of 128 bin right now), and draw a plot to the topic `drawn_histogram`.

# Python API

Long term, I'd like to be able to call this drawing function from Python,
however we're not there yet.

There IS a totally separate python node that publishes a pointcloud
for visualization in rviz:

`rosrun sonar_image_proc sonar_pointcloud.py`


# API

Sonar drawing is implemented in the [SonarDrawer](include/sonar_image_proc/SonarDrawer.h) class, which takes an instance of an [AbstractSonarInterface](include/sonar_image_proc/AbstractSonarInterface.h) and returns a cv::Mat.   SonarDrawer computes and stores pre-calculated matrices to accelerate the drawing.

A convenience function [drawSonar](include/sonar_image_proc/DrawSonar.h) is also provided.  It is a trivial wrapper which creates an instance of SonarDrawer then calls it.  Calls to drawSonar do not retain the cached matrices and are less efficient.

# Related Packages

* [liboculus](https://github.com/apl-ocean-engineering/liboculus) provides network IO and data parsing for the Oculus sonar (non-ROS).
* [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver) provides a ROS node for interfacing with the Oculus sonar.
* [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs) defines the ROS [ProjectedSonarImage](https://github.com/apl-ocean-engineering/marine_msgs/blob/main/marine_acoustic_msgs/msg/ProjectedSonarImage.msg) message type published by [oculus_sonar_driver](https://gitlab.com/apl-ocean-engineering/oculus_sonar_driver).
* [rqt_sonar_image_view](https://github.com/apl-ocean-engineering/rqt_sonar_image_view) is an Rqt plugin for displaying sonar imagery (uses [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc))


# License

Licensed under [BSD 3-clause license](LICENSE).
