#! /usr/bin/env python3
"""
Copyright 2023 University of Washington Applied Physics Laboratory
Author: Marc Micatka & Laura Lindzey
"""

from __future__ import annotations  # use type of class in member function annotation.

import numpy as np
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarImageMetadata(object):
    def __init__(self, sonar_image_msg: ProjectedSonarImage):
        """
        Metadata for a sonar image, containing all information necessary
        to compute its geometry.
        NOTE(lindzey): excludes beamwidths because those are not used when
            deciding what elevation angles to publish.
        """
        self.num_angles = len(sonar_image_msg.beam_directions)
        self.num_ranges = len(sonar_image_msg.ranges)
        self.ranges = np.array(sonar_image_msg.ranges)

        # np.min/np.max/np.median raise on an empty sequence, and a ping with
        # no ranges or no beams is something we'd rather report than crash on.
        self.min_range = float(np.min(self.ranges)) if self.num_ranges else 0.0
        self.max_range = float(np.max(self.ranges)) if self.num_ranges else 0.0

        # One pass over beam_directions instead of three list comprehensions
        beams = np.array(
            [(bd.x, bd.y, bd.z) for bd in sonar_image_msg.beam_directions],
            dtype=float,
        ).reshape(-1, 3)
        xx, yy, zz = beams[:, 0], beams[:, 1], beams[:, 2]

        self.azimuths = np.arctan2(-1 * yy, np.sqrt(xx**2 + zz**2))
        self.min_azimuth = float(np.min(self.azimuths)) if self.num_angles else 0.0
        self.max_azimuth = float(np.max(self.azimuths)) if self.num_angles else 0.0

        beamwidths = sonar_image_msg.ping_info.tx_beamwidths
        elev_beamwidth = float(np.median(beamwidths)) if len(beamwidths) else 0.0
        self.min_elevation = -0.5 * elev_beamwidth
        self.max_elevation = 0.5 * elev_beamwidth

    def __eq__(self, other: SonarImageMetadata) -> bool:
        """
        Overrides the default implementation of == and != (along with is and is not)
        Determine whether all fields are "close enough" for the
        metadata to be the same.
        """
        if not isinstance(other, SonarImageMetadata):
            return NotImplemented
        if self.num_angles != other.num_angles:
            return False
        if self.num_ranges != other.num_ranges:
            return False
        return np.allclose(
            [self.min_range, self.max_range, self.min_azimuth, self.max_azimuth],
            [other.min_range, other.max_range, other.min_azimuth, other.max_azimuth],
        )

    def __str__(self) -> str:
        """
        Overrides the default implementation of print(SonarImageMetadata)
        """
        ss = "SonarImageMetadata: {} beams, {} ranges, {:0.2f}=>{:0.2f} m, {:0.1f}=>{:0.1f} deg".format(
            self.num_angles,
            self.num_ranges,
            self.min_range,
            self.max_range,
            np.degrees(self.min_azimuth),
            np.degrees(self.max_azimuth),
        )
        return ss
