cmake_minimum_required(VERSION 3.18)
project(sonar_image_proc)

# OpenCV's CUDA-enabled package config still calls the legacy FindCUDA module.
# Select the compatibility behavior explicitly on newer CMake releases.
if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # C++ only — nvcc does not understand these flags directly
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>")
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Optional CUDA draw path (lib/GpuSonarDraw.cu): colormap LUT + bicubic fan
# remap on the GPU. The CPU path remains the fallback; without a CUDA
# toolchain the build is CPU-only exactly as before.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    set(CMAKE_CUDA_ARCHITECTURES 86-real 86-virtual CACHE STRING
      "CUDA architectures (RTX 3090 default)")
  endif()
  enable_language(CUDA)
  # stale-cache self-heal (see sonar_slam_cpp/CMakeLists.txt for rationale)
  if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 12)
    set(_cuda_arch_floor 75)
  else()
    set(_cuda_arch_floor 0)
  endif()
  set(_cuda_arch_ok TRUE)
  foreach(_a IN LISTS CMAKE_CUDA_ARCHITECTURES)
    string(REGEX MATCH "^[0-9]+" _an "${_a}")
    if(_an AND _an LESS _cuda_arch_floor)
      set(_cuda_arch_ok FALSE)
    endif()
  endforeach()
  if(NOT _cuda_arch_ok)
    message(WARNING "sonar_image_proc: cached CMAKE_CUDA_ARCHITECTURES "
      "'${CMAKE_CUDA_ARCHITECTURES}' is below the sm_${_cuda_arch_floor} floor "
      "— resetting to 86.")
    set(CMAKE_CUDA_ARCHITECTURES 86-real 86-virtual CACHE STRING "" FORCE)
  endif()
  find_package(CUDAToolkit REQUIRED)
  message(STATUS "sonar_image_proc: CUDA draw enabled (arch ${CMAKE_CUDA_ARCHITECTURES})")
else()
  message(STATUS "sonar_image_proc: no CUDA compiler found, building CPU-only")
endif()

# ament_auto resolves the ROS/ament build deps from package.xml
# (ament_cmake_python, rclcpp, rclcpp_components, rclpy, sensor_msgs, std_msgs,
#  marine_acoustic_msgs, cv_bridge, image_transport, rosbag2_cpp, rosbag2_storage).
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_depend_on_packages is the non-deprecated API in newer ament
# releases, but it is not provided by ROS 2 Jazzy. Keep the package buildable
# on both versions while Jazzy remains in use.
function(sonar_target_dependencies target)
  if(COMMAND ament_auto_depend_on_packages)
    ament_auto_depend_on_packages(${target} PACKAGES ${ARGN})
  else()
    ament_target_dependencies(${target} ${ARGN})
  endif()
endfunction()

# System (non-ament) deps found explicitly. Only bag2sonar uses Boost, and it
# uses program_options; requiring the unused filesystem/system components makes
# otherwise valid builds fail on distributions where Boost.System is
# header-only.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost COMPONENTS program_options REQUIRED)
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
sonar_target_dependencies(sonar_image_proc
  rclcpp
  sensor_msgs
  marine_acoustic_msgs
  cv_bridge
)

if(CMAKE_CUDA_COMPILER)
  target_sources(sonar_image_proc PRIVATE lib/GpuSonarDraw.cu)
  # PUBLIC: the components library keys its GPU dispatch off this define
  target_compile_definitions(sonar_image_proc PUBLIC SONAR_IMAGE_PROC_WITH_CUDA)
  target_link_libraries(sonar_image_proc CUDA::cudart_static)
endif()

# Component library containing ROS 2 components (formerly nodelets)
add_library(sonar_image_proc_components SHARED
  ros/src/draw_sonar_component.cpp
  ros/src/sonar_postprocessor_component.cpp
)
target_link_libraries(sonar_image_proc_components sonar_image_proc)
sonar_target_dependencies(sonar_image_proc_components
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
sonar_target_dependencies(draw_sonar_node
  rclcpp
)

add_executable(sonar_postprocessor_node ros/src/sonar_postprocessor_node.cpp)
target_link_libraries(sonar_postprocessor_node sonar_image_proc_components)
sonar_target_dependencies(sonar_postprocessor_node
  rclcpp
)

# bag2sonar tool for offline processing
add_executable(bag2sonar ros/tools/bag2sonar.cpp)
target_link_libraries(bag2sonar
  sonar_image_proc
  Boost::program_options
)
target_include_directories(bag2sonar PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ros/include
)
sonar_target_dependencies(bag2sonar
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  marine_acoustic_msgs
  cv_bridge
)

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
  bag2sonar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
  FILES_MATCHING PATTERN "*.yaml"
)

# Install the Python module into the active interpreter's platform-independent
# package directory instead of a hard-coded Python-version path.
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sonar_pointcloud.py
  scripts/sonar_fov.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_draw_sonar test/unit/test_draw_sonar.cpp)
  target_link_libraries(test_draw_sonar sonar_image_proc)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
