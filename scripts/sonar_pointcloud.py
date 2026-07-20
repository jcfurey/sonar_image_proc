#!/usr/bin/env python3
"""
ROS 2 Dynamic Sonar Pointcloud Node
Fully dynamic parameter-enabled version.
"""

from __future__ import annotations

import time
import threading
import numpy as np
from matplotlib import cm

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

from marine_acoustic_msgs.msg import ProjectedSonarImage, SonarImageData
from sensor_msgs.msg import PointCloud2, PointField

from sonar_image_proc.sonar_msg_metadata import SonarImageMetadata


# ======================================================================================
# Geometry Generation
# ======================================================================================

def make_geometry(sonar_msg_metadata: SonarImageMetadata, elevations) -> np.ndarray:
    """Per-elevation point geometry, laid out to match the image data
    (row-major: index = range_bin * num_angles + beam).

    Frame convention follows the message's own beam_directions
    (x = elevation, y = -sin(azimuth), z = cos(azimuth) — the driver publishes
    beam.y = -sin(az)); SonarImageMetadata.azimuths recovers the bearing
    convention via atan2(-y, ...), so y must be negated here. The previous
    version used +sin(az), which mirrored the cloud left-right relative to
    the driver's declared geometry and the sonar_proc cloud.

    Returns float32 array of shape (num_elevations, num_ranges * num_angles, 3).
    """
    ces = np.cos(elevations)
    ses = np.sin(elevations)
    cas = np.cos(sonar_msg_metadata.azimuths)
    sas = np.sin(sonar_msg_metadata.azimuths)

    # (n_elev, n_ranges, n_angles) — image layout, range-major
    r = sonar_msg_metadata.ranges[np.newaxis, :, np.newaxis]
    x = np.broadcast_to(
        (sonar_msg_metadata.ranges[np.newaxis, :] * ses[:, np.newaxis])[:, :, np.newaxis],
        (len(elevations), sonar_msg_metadata.num_ranges, sonar_msg_metadata.num_angles),
    )
    y = r * ces[:, np.newaxis, np.newaxis] * -sas[np.newaxis, np.newaxis, :]
    z = r * ces[:, np.newaxis, np.newaxis] * cas[np.newaxis, np.newaxis, :]

    points = np.stack([x, y, z], axis=-1).reshape(len(elevations), -1, 3)
    return np.ascontiguousarray(points, dtype=np.float32)


def make_color_lookup() -> np.ndarray:
    """Inferno colormap with intensity-proportional alpha, packed as
    0xAARRGGBB uint32 (the PCL 'rgba' convention RViz and Foxglove read)."""
    color_lookup = np.zeros(256, dtype=np.uint32)

    for aa in range(256):
        r, g, b, _ = cm.inferno(aa)
        color_lookup[aa] = (
            (aa << 24)                     # alpha = intensity
            | (int(255 * r) << 16)
            | (int(255 * g) << 8)
            | int(255 * b)
        )

    return color_lookup


# ======================================================================================
# Node
# ======================================================================================

class SonarPointcloud(Node):

    POINT_DTYPE = np.dtype(
        [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgba", "<u4")]
    )

    def __init__(self):

        super().__init__("sonar_pointcloud")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            ProjectedSonarImage,
            "sonar_image",
            self.sonar_image_callback,
            qos,
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            "sonar_cloud",
            qos,
        )

        # Thread safety for dynamic updates
        self.lock = threading.Lock()

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("publish_all_points", False)
        self.declare_parameter("cmin", 0.74)
        self.declare_parameter("threshold", 0.0)
        # skip points whose colormap alpha would be 0 (normalized intensity
        # <= cmin) — they render invisible but dominate the message size
        self.declare_parameter("drop_invisible", True)
        self.declare_parameter("elev_steps", 2)
        self.declare_parameter("min_elev_deg", -10.0)
        self.declare_parameter("max_elev_deg", 10.0)
        self.declare_parameter("frame_id", "")

        self._load_parameters()
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # ---------------- INTERNAL STATE ----------------
        self.geometry = None
        self.sonar_msg_metadata = None
        self.color_lookup = make_color_lookup()

        self.get_logger().info("Sonar Pointcloud node started with dynamic parameters.")

    # ==================================================================================
    # Parameter Handling
    # ==================================================================================

    def _load_parameters(self):

        self.publish_all_points = self.get_parameter(
            "publish_all_points"
        ).value

        self.cmin = float(self.get_parameter("cmin").value)
        self.threshold = float(self.get_parameter("threshold").value)
        self.drop_invisible = bool(self.get_parameter("drop_invisible").value)

        elev_steps = int(self.get_parameter("elev_steps").value)
        min_elev = np.radians(
            float(self.get_parameter("min_elev_deg").value)
        )
        max_elev = np.radians(
            float(self.get_parameter("max_elev_deg").value)
        )

        self.elevations = np.linspace(min_elev, max_elev, elev_steps)
        self.frame_id = self.get_parameter("frame_id").value

    def _on_parameter_change(self, params):

        with self.lock:

            elevation_changed = False

            for param in params:

                if param.name == "publish_all_points":
                    self.publish_all_points = param.value

                elif param.name == "cmin":
                    self.cmin = float(param.value)

                elif param.name == "threshold":
                    self.threshold = float(param.value)

                elif param.name == "drop_invisible":
                    self.drop_invisible = bool(param.value)

                elif param.name in [
                    "elev_steps",
                    "min_elev_deg",
                    "max_elev_deg",
                ]:
                    elevation_changed = True

                elif param.name == "frame_id":
                    self.frame_id = param.value

            if elevation_changed:
                self._load_parameters()
                self.geometry = None
                self.get_logger().info("Elevation parameters updated → geometry reset.")

        return SetParametersResult(successful=True)

    # ==================================================================================
    # Intensity Processing
    # ==================================================================================

    def _image_dtype(self, image: SonarImageData):

        if image.dtype == image.DTYPE_UINT8:
            return np.uint8
        if image.dtype == image.DTYPE_UINT16:
            return np.uint16
        if image.dtype == image.DTYPE_UINT32:
            return np.uint32
        # don't raise: an exception here propagates out of the callback and
        # kills the node (respawn defaults to False in the bringup launch)
        self.get_logger().error(
            f"Unsupported sonar image dtype {image.dtype} "
            "(only uint8/uint16/uint32 supported)",
            throttle_duration_sec=5.0,
        )
        return None

    # ==================================================================================
    # Callback
    # ==================================================================================

    def sonar_image_callback(self, sonar_image_msg: ProjectedSonarImage):

        with self.lock:

            begin_time = self.get_clock().now()

            header = sonar_image_msg.header
            if self.frame_id:
                header.frame_id = self.frame_id

            new_metadata = SonarImageMetadata(sonar_image_msg)

            if self.sonar_msg_metadata is None or \
               self.sonar_msg_metadata != new_metadata:

                self.sonar_msg_metadata = new_metadata
                self.geometry = make_geometry(
                    self.sonar_msg_metadata,
                    self.elevations
                )

            data_type = self._image_dtype(sonar_image_msg.image)
            if data_type is None:
                return

            raw = np.frombuffer(sonar_image_msg.image.data, dtype=data_type)
            dtype_max = np.iinfo(data_type).max
            log_max = np.log(dtype_max)

            # Selection happens in the raw domain (log is monotonic:
            # log(raw)/log(max) > t  <=>  raw > max**t), so the log runs only
            # over the selected bins. Points with normalized intensity <= cmin
            # get color index 0 => alpha 0: invisible, so by default they are
            # not worth publishing at all (this is the dominant bandwidth cost
            # with a low threshold).
            if self.publish_all_points:
                norm = np.log(np.maximum(1, raw).astype(np.float32)) / log_max
                sel_idx = None
                npts = len(raw)
            else:
                cutoff_norm = self.threshold
                if self.drop_invisible:
                    cutoff_norm = max(cutoff_norm, self.cmin)
                raw_cutoff = dtype_max ** cutoff_norm
                sel_idx = np.flatnonzero(raw > raw_cutoff)
                if len(sel_idx) == 0:
                    return
                norm = np.log(raw[sel_idx].astype(np.float32)) / log_max
                npts = len(sel_idx)

            colors = (norm - self.cmin) / (1.0 - self.cmin)
            c_uint8 = (255 * np.clip(colors, 0.0, 1.0)).astype(np.uint8)
            rgba = self.color_lookup[c_uint8]

            geometry = self.geometry if sel_idx is None else self.geometry[:, sel_idx, :]

            n_elev = len(self.elevations)
            out = np.empty(n_elev * npts, dtype=self.POINT_DTYPE)
            xyz = geometry.reshape(-1, 3)
            out["x"] = xyz[:, 0]
            out["y"] = xyz[:, 1]
            out["z"] = xyz[:, 2]
            out["rgba"] = np.tile(rgba, n_elev)

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1),
            ]

            cloud_msg = PointCloud2(
                header=header,
                height=1,
                width=len(out),
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=16,
                row_step=16 * len(out),
                data=out.tobytes(),
            )

            self.publisher.publish(cloud_msg)

            elapsed = (self.get_clock().now() - begin_time).nanoseconds / 1e9
            self.get_logger().debug(
                f"Published {len(out)} pts "
                f"in {elapsed:.3f}s"
            )


# ======================================================================================
# Main
# ======================================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SonarPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()