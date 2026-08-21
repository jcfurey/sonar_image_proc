# Pixel geometry for a Cartesian, orthographic imaging-sonar fan.
#
# This is deliberately not sensor_msgs/CameraInfo: a fan is a metric plan-view
# remap of range and bearing samples, not a perspective pinhole image.  The
# header stamp and frame_id must exactly match the sensor_msgs/Image this
# message describes.
std_msgs/Header header

# Dimensions of the Cartesian fan image in pixels.
uint32 width
uint32 height

# Sonar apex in image coordinates.  Pixel coordinates use the OpenCV/ROS image
# convention (+u right, +v down).  The apex commonly lies on the lower image
# boundary, so origin_v == height is valid even though it is outside the final
# row's pixel centre.
float64 origin_u
float64 origin_v

# Isotropic scale of the Cartesian remap.
float64 pixels_per_meter

# Physical bounds represented by the source ping.  Bearings are radians and
# increase toward +u in the published fan image.
float64 min_range
float64 max_range
float64 min_bearing
float64 max_bearing
