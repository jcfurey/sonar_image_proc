# Pixel geometry for the rectangular range-bearing sonar image.
#
# This is deliberately not sensor_msgs/CameraInfo.  Its horizontal coordinate
# is rectilinear, like a pinhole camera, but its vertical coordinate is measured
# sonar range (far at the top, near at the bottom), not elevation.  The header
# stamp and frame_id exactly match the sensor_msgs/Image this message describes.
std_msgs/Header header

uint32 width
uint32 height

# Horizontal bearing projection, in pixels and radians:
#
#   u = horizontal_focal_length * tan(bearing) + principal_point_u
#   bearing = atan((u - principal_point_u) / horizontal_focal_length)
float64 horizontal_focal_length
float64 principal_point_u

# Vertical range projection, in pixels and metres:
#
#   range = max_range - v * meters_per_row
float64 meters_per_row
float64 min_range
float64 max_range

# Physical bearing bounds represented in the image, in radians. They map to
# the centres of the first and last columns respectively.
float64 min_bearing
float64 max_bearing
