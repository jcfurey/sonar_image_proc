// Copyright 2026 ERDC

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sonar_image_proc
{

// Convert a marine_acoustic_msgs beam-major payload into the range-major
// working layout used by the existing CPU and CUDA drawing implementations.
// bytes_per_cell supports all SonarImageData dtypes without host-endian
// reinterpretation.
inline bool beamMajorToRangeMajor(
    const std::vector<std::uint8_t>& input,
    std::size_t range_count,
    std::size_t beam_count,
    std::size_t bytes_per_cell,
    std::vector<std::uint8_t>& output)
{
  if (range_count == 0 || beam_count == 0 || bytes_per_cell == 0 ||
      range_count > input.max_size() / beam_count) {
    return false;
  }
  const std::size_t cell_count = range_count * beam_count;
  if (cell_count > input.max_size() / bytes_per_cell) {
    return false;
  }
  const std::size_t byte_count = cell_count * bytes_per_cell;
  if (input.size() < byte_count) {
    return false;
  }

  output.resize(byte_count);
  for (std::size_t beam = 0; beam < beam_count; ++beam) {
    for (std::size_t range = 0; range < range_count; ++range) {
      const std::size_t src = (beam * range_count + range) * bytes_per_cell;
      const std::size_t dst = (range * beam_count + beam) * bytes_per_cell;
      std::copy_n(input.data() + src, bytes_per_cell, output.data() + dst);
    }
  }
  return true;
}

}  // namespace sonar_image_proc
