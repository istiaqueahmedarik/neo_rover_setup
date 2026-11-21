#!/usr/bin/env python3
"""Command watchdog, RealSense D435 stereo calibration/pointcloud, and RViz ArUco MarkerArray."""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]
BASELINE_M = 0.050
QOS_RELIABLE = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
QOS_TRANSIENT = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Runtime(Node):
    """Watchdog for /cmd_vel plus D435 stereo CameraInfo, RGB-D cloud, and /aruco/markers."""

    def __init__(self):
        super().__init__('rover_runtime')
        self.last = -1e9
        self.cmd = Twist()
        self.depth_k = None
        self.latest_infra1 = None

        self.pub = self.create_publisher(Twist, '/simulation/cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel', self.receive, 1)
        self.create_timer(0.05, self.tick)

        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', QOS_RELIABLE)
        self.infra1_info_pub = self.create_publisher(CameraInfo, '/camera/infra1/camera_info', QOS_RELIABLE)
        self.infra2_info_pub = self.create_publisher(CameraInfo, '/camera/infra2/camera_info', QOS_RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', QOS_RELIABLE)
        self.aruco_pub = self.create_publisher(MarkerArray, '/aruco/markers', QOS_TRANSIENT)

        self.create_subscription(CameraInfo, '/gz_realsense/depth/camera_info', self.on_depth_info, QOS_RELIABLE)
        self.create_subscription(CameraInfo, '/gz_realsense/infra2/camera_info', self.on_infra2_info, QOS_RELIABLE)
        self.create_subscription(Image, '/camera/infra1/image_raw', self.on_infra1, QOS_RELIABLE)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, QOS_RELIABLE)

        self.aruco_msg = self._build_aruco_markers()
        if self.aruco_msg.markers:
            self.aruco_pub.publish(self.aruco_msg)
            self.create_timer(2.0, self._publish_aruco)

    def _build_aruco_markers(self) -> MarkerArray:
        arr = MarkerArray()
        info_path = ROOT / 'gazebo/build_info.json'
        if not info_path.exists():
            return arr
        info = json.loads(info_path.read_text())
        sx, sy, sz = info.get('spawn_xyz', [0.0, 0.0, 0.15])
        ground_spawn = sz - 0.15
        for idx, m in enumerate(info.get('aruco_markers', [])):
            wx, wy, wz = m['world_xyz']
            bw, bh = m['board_size_m']
            yaw = math.radians(float(m['yaw_deg']))
            ox, oy, oz = wx - sx, wy - sy, wz - ground_spawn

            panel = Marker()
            panel.header.frame_id = 'odom'
            panel.ns = 'aruco_panel'
            panel.id = idx
            panel.type = Marker.CUBE
            panel.action = Marker.ADD
            panel.pose.position.x = ox
            panel.pose.position.y = oy
            panel.pose.position.z = oz
            panel.pose.orientation.z = math.sin(yaw * 0.5)
            panel.pose.orientation.w = math.cos(yaw * 0.5)
            panel.scale.x = 0.02
            panel.scale.y = float(bw)
            panel.scale.z = float(bh)
            panel.color.r = 0.15
            panel.color.g = 0.90
            panel.color.b = 0.95
            panel.color.a = 0.95
            arr.markers.append(panel)

            label = Marker()
            label.header.frame_id = 'odom'
            label.ns = 'aruco_label'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = ox
            label.pose.position.y = oy
            label.pose.position.z = oz + float(bh) * 0.5 + 0.28
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 0.96
            label.color.b = 0.72
            label.color.a = 1.0
            sw, sh = m['size_cm']
            label.text = f"{m['name']}\nID:{m['id']} {m['dictionary']} ({sw:g}x{sh:g}cm)"
            arr.markers.append(label)
        return arr

    def _publish_aruco(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for mk in self.aruco_msg.markers:
            mk.header.stamp = stamp
        self.aruco_pub.publish(self.aruco_msg)

    def receive(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            return
        self.cmd = Twist()
        self.cmd.linear.x = max(-0.8, min(0.8, msg.linear.x))
        self.cmd.angular.z = max(-1.5, min(1.5, msg.angular.z))
        self.last = time.monotonic()

    def tick(self) -> None:
        self.pub.publish(self.cmd if time.monotonic() - self.last < 0.5 else Twist())

    def on_depth_info(self, msg: CameraInfo) -> None:
        self.depth_k = list(msg.k)
        msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_info_pub.publish(msg)
        self.infra1_info_pub.publish(msg)

    def on_infra2_info(self, msg: CameraInfo) -> None:
        msg.header.frame_id = 'camera_infra2_optical_frame'
        msg.p[3] = -msg.p[0] * BASELINE_M
        self.infra2_info_pub.publish(msg)

    def on_infra1(self, msg: Image) -> None:
        self.latest_infra1 = msg

    def on_depth(self, msg: Image) -> None:
        if self.depth_k is None or self.cloud_pub.get_subscription_count() == 0:
            return
        depth = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4').reshape(
            msg.height, msg.step // 4
        )[:, :msg.width][::2, ::2].copy()
        valid = np.isfinite(depth) & (depth >= 0.2) & (depth <= 15.0)
        z = np.where(valid, depth, np.nan).astype(np.float32)

        h, w = z.shape
        v, u = np.mgrid[0:msg.height:2, 0:msg.width:2]
        fx, cx, fy, cy = self.depth_k[0], self.depth_k[2], self.depth_k[4], self.depth_k[5]
        x = ((u - cx) * z / fx).astype(np.float32)
        y = ((v - cy) * z / fy).astype(np.float32)

        im = self.latest_infra1
        if im is not None and im.height == msg.height and im.width == msg.width:
            bpp = max(1, im.step // im.width)
            arr = np.frombuffer(im.data, dtype=np.uint8).reshape(im.height, im.step)[:, :im.width * bpp].reshape(
                im.height, im.width, bpp
            )[::2, ::2]
            if bpp >= 3:
                if im.encoding == 'bgr8':
                    r, g, b = arr[:, :, 2], arr[:, :, 1], arr[:, :, 0]
                else:
                    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
            else:
                r = g = b = arr[:, :, 0]
            packed = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
        else:
            packed = np.full((h, w), 0xFFFFFF, dtype=np.uint32)
        rgb = packed.view(np.float32)

        pts = np.stack((x, y, z, rgb), axis=-1).astype('<f4')
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.header.frame_id = 'camera_depth_optical_frame'
        cloud.height = h
        cloud.width = w
        cloud.fields = [
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z', 'rgb'))
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * w
        cloud.is_dense = False
        cloud.data = pts.tobytes()
        self.cloud_pub.publish(cloud)


def main():
    rclpy.init()
    n = Runtime()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            n.pub.publish(Twist())
            n.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
