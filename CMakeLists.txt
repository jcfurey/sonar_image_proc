cmake_minimum_required(VERSION 3.8)
project(sonar_image_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(Boost COMPONENTS filesystem system program_options REQUIRED)
list(APPEND OPENCV_COMPONENTS core highgui imgproc)
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

find_package(Boost COMPONENTS filesystem system program_options REQUIRED)
list(APPEND OPENCV_COMPONENTS core highgui imgproc)
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

include_directories(
  include/
  ros/include/
  ${OpenCV_INCLUDE_DIRS}
)

set(sonar_image_proc_SRCS
  lib/AbstractSonarInterface.cpp
  lib/ColorMaps.cpp
  lib/SonarDrawer.cpp
  lib/OldDrawSonar.cpp
  lib/HistogramGenerator.cpp
)

# Core library containing core functionality
add_library(sonar_image_proc SHARED ${sonar_image_proc_SRCS})
target_link_libraries(sonar_image_proc ${OpenCV_LIBS})
ament_target_dependencies(sonar_image_proc
  rclcpp
  sensor_msgs
  marine_acoustic_msgs
  cv_bridge
)

# Component library containing ROS 2 components (formerly nodelets)
add_library(sonar_image_proc_components SHARED
  ros/src/draw_sonar_component.cpp
  ros/src/sonar_postprocessor_component.cpp
)
target_link_libraries(sonar_image_proc_components sonar_image_proc)
ament_target_dependencies(sonar_image_proc_components
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  marine_acoustic_msgs
  cv_bridge
  image_transport
)

# Register components
rclcpp_components_register_nodes(sonar_image_proc_components "draw_sonar::DrawSonarComponent")
rclcpp_components_register_nodes(sonar_image_proc_components "sonar_postprocessor::SonarPostprocessorComponent")

# Standalone nodes
add_executable(draw_sonar_node ros/src/draw_sonar_node.cpp)
target_link_libraries(draw_sonar_node sonar_image_proc_components)
ament_target_dependencies(draw_sonar_node
  rclcpp
)

add_executable(sonar_postprocessor_node ros/src/sonar_postprocessor_node.cpp)
target_link_libraries(sonar_postprocessor_node sonar_image_proc_components)
ament_target_dependencies(sonar_postprocessor_node
  rclcpp
)

# TODO: Port bag2sonar tool to rosbag2 API
# add_executable(bag2sonar ros/tools/bag2sonar.cpp)
# target_link_libraries(bag2sonar sonar_image_proc)
# ament_target_dependencies(bag2sonar
#   rclcpp
#   rosbag2_cpp
#   sensor_msgs
#   marine_acoustic_msgs
# )

# Install targets
install(TARGETS
  sonar_image_proc
  sonar_image_proc_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  draw_sonar_node
  sonar_postprocessor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY ros/include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install Python modules and scripts
install(DIRECTORY python/sonar_image_proc
  DESTINATION lib/python3.12/site-packages
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  scripts/sonar_pointcloud.py
  scripts/sonar_fov.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
