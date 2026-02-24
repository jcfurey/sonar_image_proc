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

    idxs = np.arange(
        0,
        sonar_msg_metadata.num_angles * sonar_msg_metadata.num_ranges,
    )
    idxs = idxs.reshape(
        sonar_msg_metadata.num_ranges,
        sonar_msg_metadata.num_angles,
    ).flatten(order="F")

    ces = np.cos(elevations)
    ses = np.sin(elevations)
    cas = np.cos(sonar_msg_metadata.azimuths)
    sas = np.sin(sonar_msg_metadata.azimuths)

    new_shape = (
        len(elevations),
        sonar_msg_metadata.num_ranges * sonar_msg_metadata.num_angles,
        3,
    )

    points = np.zeros(new_shape)

    x_temp = np.tile(
        sonar_msg_metadata.ranges[np.newaxis, :] * ses[:, np.newaxis],
        reps=sonar_msg_metadata.num_angles,
    ).flatten()

    y_temp = (
        sonar_msg_metadata.ranges[np.newaxis, np.newaxis, :]
        * ces[:, np.newaxis, np.newaxis]
        * sas[np.newaxis, :, np.newaxis]
    ).flatten()

    z_temp = (
        sonar_msg_metadata.ranges[np.newaxis, np.newaxis, :]
        * ces[:, np.newaxis, np.newaxis]
        * cas[np.newaxis, :, np.newaxis]
    ).flatten()

    points[:, idxs, :] = np.stack([x_temp, y_temp, z_temp], axis=1).reshape(new_shape)

    return points


def make_color_lookup() -> np.ndarray:

    color_lookup = np.zeros((256, 4), dtype=np.float32)

    for aa in range(256):
        r, g, b, _ = cm.inferno(aa)
        alpha = aa / 256.0
        color_lookup[aa, :] = [r, g, b, alpha]

    return color_lookup


# ======================================================================================
# Node
# ======================================================================================

class SonarPointcloud(Node):

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

    def normalize_intensity_array(self, image: SonarImageData):

        if image.dtype == image.DTYPE_UINT8:
            data_type = np.uint8
        elif image.dtype == image.DTYPE_UINT32:
            data_type = np.uint32
        else:
            raise Exception("Only 8 bit and 32 bit data supported")

        intensities = np.frombuffer(image.data, dtype=data_type)
        new_intensities = intensities.astype(np.float32)

        return np.log(np.maximum(1, new_intensities)) / np.log(
            np.iinfo(data_type).max
        )

    # ==================================================================================
    # Callback
    # ==================================================================================

    def sonar_image_callback(self, sonar_image_msg: ProjectedSonarImage):

        with self.lock:

            begin_time = time.time()

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

            normalized_intensities = self.normalize_intensity_array(
                sonar_image_msg.image
            )

            if self.publish_all_points:
                selected_intensities = normalized_intensities
                geometry = self.geometry
            else:
                pos_idx = np.where(normalized_intensities > self.threshold)
                selected_intensities = normalized_intensities[pos_idx]
                geometry = self.geometry[:, pos_idx[0]]

            if len(selected_intensities) == 0:
                return

            colors = (selected_intensities - self.cmin) / (1.0 - self.cmin)
            c_clipped = np.clip(colors, 0.0, 1.0)
            c_uint8 = (255 * c_clipped).astype(np.uint8)

            npts = len(selected_intensities)

            output_points = np.zeros(
                (len(self.elevations) * npts, 7),
                dtype=np.float32,
            )

            color_vals = self.color_lookup[c_uint8]

            elev_points = np.empty((npts, 7), dtype=np.float32)
            elev_points[:, 3:] = color_vals

            for i in range(len(self.elevations)):
                elev_points[:, 0:3] = geometry[i, :, :]
                start = i * npts
                output_points[start:start + npts, :] = elev_points

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1),
                PointField(name="a", offset=24, datatype=PointField.FLOAT32, count=1),
            ]

            cloud_msg = PointCloud2(
                header=header,
                height=1,
                width=len(output_points),
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=7 * 4,
                row_step=7 * 4 * len(output_points),
                data=output_points.tobytes(),
            )

            self.publisher.publish(cloud_msg)

            self.get_logger().debug(
                f"Published {len(output_points)} pts "
                f"in {time.time() - begin_time:.3f}s"
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