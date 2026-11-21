#!/usr/bin/env python3
"""Verify 3D LiDAR, RealSense D435 stereo/RGB-D, ArUco markers, TF, chassis motion and watchdog."""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

R = Path(__file__).resolve().parents[1]
QOS_RELIABLE = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
QOS_TRANSIENT = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def decode_rgb_image(msg: Image) -> np.ndarray:
    """Decode ROS Image (rgb8/bgr8/mono8) into uint8 RGB array."""
    bpp = max(1, msg.step // msg.width)
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * bpp]
    if bpp >= 3:
        img = arr.reshape(msg.height, msg.width, bpp)[:, :, :3].copy()
        if msg.encoding == 'bgr8':
            img = img[:, :, ::-1].copy()
        return img
    gray = arr.reshape(msg.height, msg.width)
    return np.repeat(gray[:, :, None], 3, axis=2)


def save_image_preview(msg: Image, path: Path) -> None:
    """Decode ROS Image and write PNG preview."""
    PILImage.fromarray(decode_rgb_image(msg)).save(path)


def detect_aruco(rgb_img: np.ndarray, dict_name: str = 'DICT_4X4_50'):
    """Detect ArUco IDs in an RGB image and return (ids_list, corners)."""
    gray = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    id_list = [int(x) for x in ids.ravel()] if ids is not None else []
    return id_list, corners


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', '78')
    os.environ.setdefault('GZ_PARTITION', 'my_rover_test')
    log = (R / 'logs/lidar_simulator.log').open('w')
    proc = subprocess.Popen(
        [str(R / 'start.sh')] + ([] if args.gui else ['--headless']),
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    n = rclpy.create_node('lidar_and_stereo_verification')
    latest = {}
    first_spawn_color = {'msg': None}
    count = {}
    buffer = Buffer()
    TransformListener(buffer, n)

    def cb(k: str):
        def f(m):
            if k == 'cam_color' and first_spawn_color['msg'] is None:
                first_spawn_color['msg'] = m
            latest[k] = m
            count[k] = count.get(k, 0) + 1
        return f

    sensor_subs = [
        ('clock', '/clock', Clock, qos_profile_sensor_data),
        ('odom', '/odom', Odometry, qos_profile_sensor_data),
        ('truth', '/simulation/ground_truth', Odometry, qos_profile_sensor_data),
        ('cloud', '/lidar/points', PointCloud2, qos_profile_sensor_data),
        ('cloud_reliable', '/lidar/points', PointCloud2, QOS_RELIABLE),
        ('joints', '/joint_states', JointState, qos_profile_sensor_data),
        ('imu', '/imu/data', Imu, qos_profile_sensor_data),
        ('cam_color', '/camera/color/image_raw', Image, QOS_RELIABLE),
        ('cam_color_info', '/camera/color/camera_info', CameraInfo, QOS_RELIABLE),
        ('cam_infra1', '/camera/infra1/image_raw', Image, QOS_RELIABLE),
        ('cam_infra1_info', '/camera/infra1/camera_info', CameraInfo, QOS_RELIABLE),
        ('cam_infra2', '/camera/infra2/image_raw', Image, QOS_RELIABLE),
        ('cam_infra2_info', '/camera/infra2/camera_info', CameraInfo, QOS_RELIABLE),
        ('cam_depth', '/camera/depth/image_raw', Image, QOS_RELIABLE),
        ('cam_depth_info', '/camera/depth/camera_info', CameraInfo, QOS_RELIABLE),
        ('cam_points', '/camera/depth/color/points', PointCloud2, QOS_RELIABLE),
        ('aruco_markers', '/aruco/markers', MarkerArray, QOS_TRANSIENT),
    ]
    for k, t, ty, qos in sensor_subs:
        n.create_subscription(ty, t, cb(k), qos)
    pub = n.create_publisher(Twist, '/cmd_vel', 1)

    def spin(seconds: float, v=None, w: float = 0.0) -> None:
        end = time.monotonic() + seconds
        next_cmd = 0.0
        while time.monotonic() < end:
            if proc.poll() is not None:
                raise RuntimeError((R / 'logs/lidar_simulator.log').read_text()[-3000:])
            if v is not None and time.monotonic() >= next_cmd:
                m = Twist()
                m.linear.x = float(v)
                m.angular.z = float(w)
                pub.publish(m)
                next_cmd = time.monotonic() + 0.05
            rclpy.spin_once(n, timeout_sec=0.005)

    def pos() -> np.ndarray:
        p = latest['truth'].pose.pose.position
        return np.array([p.x, p.y, p.z])

    def yaw() -> float:
        q = latest['truth'].pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def stamp() -> float:
        t = latest['clock'].clock
        return t.sec + t.nanosec / 1e9

    try:
        deadline = time.monotonic() + 75
        while len(latest) < len(sensor_subs) and time.monotonic() < deadline:
            spin(0.1)
        assert len(latest) == len(sensor_subs), f'Missing topics: {set(k for k, *_ in sensor_subs) - set(latest)} ({count})'

        spin(2.0, 0.0)
        spawn_rgb = decode_rgb_image(first_spawn_color['msg'])
        spawn_ids, spawn_corners = detect_aruco(spawn_rgb, 'DICT_4X4_50')
        annot_bgr = cv2.cvtColor(spawn_rgb, cv2.COLOR_RGB2BGR)
        if spawn_ids:
            cv2.aruco.drawDetectedMarkers(annot_bgr, spawn_corners, np.array(spawn_ids, dtype=np.int32).reshape(-1, 1))
        cv2.imwrite(str(R / 'assets/aruco_detection_preview.png'), annot_bgr)

        p0 = pos()
        heading = yaw()
        c0 = count['cloud_reliable']
        c0_be = count['cloud']
        cam0 = count['cam_infra1']
        s0 = stamp()
        w0 = time.monotonic()
        raw0 = bytes(latest['cloud_reliable'].data)

        spin(4.0, 0.3)
        p1 = pos()
        spin(1.0, 0.0)
        y0 = yaw()
        spin(3.0, 0.0, 0.5)
        y1 = yaw()
        spin(1.5)
        p2 = pos()
        spin(1.5)
        p3 = pos()
        elapsed = time.monotonic() - w0

        # 3D LiDAR inspection
        m = latest['cloud_reliable']
        fields = {f.name: f for f in m.fields}
        assert all(fields[k].datatype == 7 for k in ['x', 'y', 'z']), m.fields
        dt = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['>f4' if m.is_bigendian else '<f4'] * 3,
            'offsets': [fields[k].offset for k in ['x', 'y', 'z']],
            'itemsize': m.point_step,
        })
        records = np.ndarray((m.height, m.width), dtype=dt, buffer=m.data, strides=(m.row_step, m.point_step))
        xyz = np.stack([records[k].ravel() for k in ['x', 'y', 'z']], axis=1)
        finite = np.isfinite(xyz).all(axis=1)
        valid = xyz[finite]
        ranges = np.linalg.norm(valid, axis=1)
        elev = np.degrees(np.arctan2(valid[:, 2], np.linalg.norm(valid[:, :2], axis=1)))
        az = np.arctan2(valid[:, 1], valid[:, 0])
        angles = np.unique(np.round(elev, 2))
        bins = np.unique(np.minimum(7, ((az + np.pi) / (2 * np.pi) * 8).astype(int)))
        near15 = valid[np.linalg.norm(valid[:, :2], axis=1) <= 15.0]
        near15_above = int((near15[:, 2] > 0.0).sum()) if len(near15) else 0
        near15_z_ext = float(np.ptp(near15[:, 2])) if len(near15) else 0.0

        # RealSense D435 stereo / depth / colored cloud inspection
        infra1 = latest['cam_infra1']
        infra2 = latest['cam_infra2']
        color = latest['cam_color']
        d_msg = latest['cam_depth']
        d_arr = np.frombuffer(d_msg.data, dtype='>f4' if d_msg.is_bigendian else '<f4').reshape(
            d_msg.height, d_msg.step // 4
        )[:, : d_msg.width]
        d_fin = np.isfinite(d_arr) & (d_arr > 0.0) & (d_arr < 100.0)
        d_frac = float(d_fin.mean())
        d_med = float(np.median(d_arr[d_fin])) if d_fin.any() else 0.0

        cam_cloud = latest['cam_points']
        cam_fields = {f.name: f for f in cam_cloud.fields}
        dt_cam = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['>f4' if cam_cloud.is_bigendian else '<f4'] * 3,
            'offsets': [cam_fields[k].offset for k in ['x', 'y', 'z']],
            'itemsize': cam_cloud.point_step,
        })
        cam_rec = np.ndarray(
            (cam_cloud.height, cam_cloud.width),
            dtype=dt_cam,
            buffer=cam_cloud.data,
            strides=(cam_cloud.row_step, cam_cloud.point_step),
        )
        cam_xyz = np.stack([cam_rec[k].ravel() for k in ['x', 'y', 'z']], axis=1)
        cam_finite_count = int(np.isfinite(cam_xyz).all(axis=1).sum())

        info2 = latest['cam_infra2_info']
        stereo_baseline_m = float(-info2.p[3] / info2.p[0])

        tf_lidar = buffer.lookup_transform('base_link', 'lidar_link', rclpy.time.Time()).transform.translation
        tf_infra2 = buffer.lookup_transform('camera_depth_frame', 'camera_infra2_frame', rclpy.time.Time()).transform.translation
        tf_color = buffer.lookup_transform('camera_depth_frame', 'camera_color_frame', rclpy.time.Time()).transform.translation

        save_image_preview(color, R / 'assets/realsense_color_preview.png')
        save_image_preview(infra1, R / 'assets/realsense_infra1_preview.png')
        save_image_preview(infra2, R / 'assets/realsense_infra2_preview.png')

        aruco_cfg = yaml.safe_load((R / 'gazebo/aruco_markers.yaml').read_text())
        expected_aruco_count = len(aruco_cfg.get('markers', []))
        marker_array = latest['aruco_markers'].markers
        panel_markers = [mk for mk in marker_array if mk.ns == 'aruco_panel']
        build_info = json.loads((R / 'gazebo/build_info.json').read_text())

        topics = sorted(n.get_topic_names_and_types())
        report = {
            'sensor': 'Gazebo gpu_lidar (32x1024, 360deg, -30..15deg, 0.2..80m) + Intel RealSense D435 stereo/RGB-D (640x480, 50mm baseline)',
            'gui_launched': args.gui,
            'cloud_frame': m.header.frame_id,
            'cloud_shape': [m.height, m.width],
            'point_count': m.width * m.height,
            'finite_point_count': int(finite.sum()),
            'finite_fraction': float(finite.mean()),
            'valid_vertical_layers': len(angles),
            'vertical_angles_deg': angles.tolist(),
            'azimuth_octants_with_returns': len(bins),
            'range_min_max_m': [float(ranges.min()), float(ranges.max())],
            'point_z_extent_m': float(np.ptp(valid[:, 2])),
            'near_15m_points_above_sensor': near15_above,
            'near_15m_z_extent_m': near15_z_ext,
            'best_effort_cloud_received': count.get('cloud', 0) > 0,
            'reliable_cloud_received': count.get('cloud_reliable', 0) > 0,
            'mount_base_to_lidar_xyz_m': [tf_lidar.x, tf_lidar.y, tf_lidar.z],
            'cloud_changed_after_motion': bytes(m.data) != raw0,
            'observed_lidar_hz': (count['cloud_reliable'] - c0) / elapsed,
            'observed_lidar_best_effort_hz': (count['cloud'] - c0_be) / elapsed,
            'observed_camera_hz': (count['cam_infra1'] - cam0) / elapsed,
            'realsense_d435': {
                'color_size': [color.width, color.height],
                'infra1_size': [infra1.width, infra1.height],
                'infra2_size': [infra2.width, infra2.height],
                'infra_stereo_images_differ': bytes(infra1.data) != bytes(infra2.data),
                'infra2_projection_baseline_m': stereo_baseline_m,
                'depth_finite_fraction': d_frac,
                'depth_median_m': d_med,
                'pointcloud_frame': cam_cloud.header.frame_id,
                'pointcloud_fields': list(cam_fields.keys()),
                'pointcloud_finite_points': cam_finite_count,
                'tf_depth_to_infra2_xyz_m': [tf_infra2.x, tf_infra2.y, tf_infra2.z],
                'tf_depth_to_color_xyz_m': [tf_color.x, tf_color.y, tf_color.z],
            },
            'aruco_detected_at_spawn_ids': spawn_ids,
            'aruco_marker_array_count': len(panel_markers),
            'expected_aruco_count': expected_aruco_count,
            'tunnel_info': build_info.get('underground_tunnel', {}),
            'real_time_factor': (stamp() - s0) / elapsed,
            'actual_forward_displacement_m': float(np.dot((p1 - p0)[:2], [math.cos(heading), math.sin(heading)])),
            'actual_turn_yaw_rad': math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)),
            'timeout_drift_m': float(np.linalg.norm(p3 - p2)),
            'message_counts': count,
            'topics': topics,
        }
        np.save(R / 'assets/lidar_points_preview.npy', valid)
        (R / 'logs/lidar_validation.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))

        assert m.header.frame_id == 'lidar_link' and report['point_count'] == 32768, report
        assert report['best_effort_cloud_received'] and report['reliable_cloud_received'], report
        assert report['valid_vertical_layers'] >= 10 and len(bins) == 8 and report['point_z_extent_m'] > 0.5, report
        assert report['near_15m_points_above_sensor'] >= 50 and report['near_15m_z_extent_m'] >= 1.5, report
        assert 0.19 <= ranges.min() and ranges.max() <= 80.1 and report['finite_fraction'] > 0.1, report
        assert abs(tf_lidar.z - 0.25) < 1e-6 and report['cloud_changed_after_motion'], report

        rs = report['realsense_d435']
        assert rs['infra1_size'] == [640, 480] and rs['infra2_size'] == [640, 480], rs
        assert rs['infra_stereo_images_differ'], rs
        assert abs(rs['infra2_projection_baseline_m'] - 0.050) < 1e-5, rs
        assert rs['depth_finite_fraction'] > 0.1 and 0.2 <= rs['depth_median_m'] <= 20.0, rs
        assert rs['pointcloud_frame'] == 'camera_depth_optical_frame' and 'rgb' in rs['pointcloud_fields'], rs
        assert rs['pointcloud_finite_points'] > 5000, rs
        assert abs(tf_infra2.y + 0.050) < 1e-6 and abs(tf_color.y - 0.015) < 1e-6, rs
        assert report['observed_camera_hz'] > 4.0, report

        assert 0 in spawn_ids, f'Expected ArUco ID 0 at spawn, got {spawn_ids}'
        assert len(panel_markers) == expected_aruco_count, report

        assert report['actual_forward_displacement_m'] > 0.5 and abs(report['actual_turn_yaw_rad']) > 0.4, report
        assert report['timeout_drift_m'] < 0.15 and report['observed_lidar_hz'] > 4.0, report
        if args.gui:
            subprocess.run([sys.executable, str(R / 'gazebo/capture_window.py')], check=True, timeout=5)
        print('PASS: 3D LiDAR, RealSense D435 stereo/depth/points, live ArUco detection, TF, motion & braking')
    finally:
        pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        log.close()


if __name__ == '__main__':
    main()
