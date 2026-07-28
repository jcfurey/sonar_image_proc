// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "sonar_image_proc/AbstractSonarInterface.h"

#include <algorithm>
#include <utility>

namespace sonar_image_proc {

const Bounds_t UnsetBounds = Bounds_t(-1, -1);

AbstractSonarInterface::AbstractSonarInterface()
    : _rangeBounds(UnsetBounds), _azimuthBounds(UnsetBounds) {}

Bounds_t AbstractSonarInterface::azimuthBounds() const {
  checkAzimuthBounds();
  return _azimuthBounds;
}

Bounds_t AbstractSonarInterface::rangeBounds() const {
  checkRangeBounds();
  return _rangeBounds;
}

void AbstractSonarInterface::checkRangeBounds() const {
  if (_rangeBounds == UnsetBounds) {
    const auto &r = ranges();

    // minmax_element on an empty range returns (end(), end()); dereferencing
    // those is undefined behavior.  A ping with no range bins has no bounds.
    if (r.empty()) {
      _rangeBounds = std::make_pair(0.0f, 0.0f);
      _maxRangeSquared = 0.0f;
      return;
    }

    auto results = std::minmax_element(r.begin(), r.end());
    _rangeBounds = std::make_pair(*(results.first), *(results.second));

    _maxRangeSquared = _rangeBounds.second * _rangeBounds.second;
  }
}

void AbstractSonarInterface::checkAzimuthBounds() const {
  if (_azimuthBounds == UnsetBounds) {
    const auto &a = azimuths();

    if (a.empty()) {
      _azimuthBounds = std::make_pair(0.0f, 0.0f);
      _minAzimuthTan = 0.0f;
      _maxAzimuthTan = 0.0f;
      return;
    }

    auto results = std::minmax_element(a.begin(), a.end());
    _azimuthBounds = std::make_pair(*(results.first), *(results.second));

    _minAzimuthTan = std::tan(_azimuthBounds.first);
    _maxAzimuthTan = std::tan(_azimuthBounds.second);
  }
}

}  // namespace sonar_image_proc
