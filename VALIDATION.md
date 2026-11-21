# Validation Report — 2026-11-21

Stack: **Gazebo Jetty 10.5 (`gz-sim 10`)**, official `leo_description` (`3.2.0`) / `leo_gz_plugins`, DART Bullet collision, and Ogre2 rendering on the 2 km rough MDRS terrain with `start_station`, `underground_tunnel`, and YAML-configurable `aruco_markers` ([aruco_markers.yaml](gazebo/aruco_markers.yaml)).

> [!NOTE]
> `pal-robotics/realsense_gazebo_plugin` (`librealsense_gazebo_plugin.so`) targets Gazebo Classic (`gazebo_ros`) and cannot load in Gazebo Jetty 10.5. The rover implements the same D435 `camera_link` / optical frame layout, `50 mm` stereo baseline, and `/camera/*` ROS 2 topics natively using Gazebo Jetty Ogre2 sensors, `ros_gz_bridge`, and [runtime.py](gazebo/runtime.py).

## 1. Current 14 m Underground Tunnel & Front-Sidebar Markers: SDF & Static Geometry Check (PASS)

After enlarging the underground tunnel (`14.0 m` bore, `3.40 m` clear width, `2.20 m` headroom, `1.80 m` underground depth with `10.0 m` entry/exit ramps) and moving start markers `ID 10` & `ID 11` directly in front of the two entrance sidebars (`x = 36.665, y = -31.80 / -36.20`):

- `.venv/bin/python gazebo/build_world.py`: **PASS** (`Built rough MDRS world (420 rocks), Start Station, 14m Underground Tunnel, and 8 ArUco markers`)
- `gz sdf -k gazebo/mdrs.sdf`: **PASS** (`Valid.`)
- Static geometry audit against `gazebo/build_info.json`, `gazebo/meshes/tunnel_overburden.stl`, and `gazebo/mdrs.sdf`: **PASS** (overburden mesh `3032` tris clears bore ceiling `12.250 m` with `12.350 m` minimum z; front sidebar end-pillars at `x = 36.68..37.04` align cleanly behind `ID 10, 11`).
- *(Live driving traversal through the enlarged 14 m tunnel was deferred because an interactive `./start.sh` session held `.simulation.lock`.)*

## 2. Graphical Integration Test: PASS

Recorded from `.venv/bin/python gazebo/verify_gazebo.py --gui` (`logs/lidar_validation.json`, run cleanly prior to enlarging the tunnel from `7 m` to `14 m`; checks 3D LiDAR, RealSense D435 stereo/depth/points, TF, spawn ArUco detection, `/aruco/markers`, `/cmd_vel` motion, watchdog braking, and shutdown):

| Check | Result |
|---|---|
| LiDAR configuration | 32 channels × 1024 samples; 360°; vertical `-30°..15°`; `0.2..80 m` |
| LiDAR finite returns in final scan | `25406 / 32768` (`77.53%`) |
| Distinct LiDAR vertical layers / octants | `32 / 32` layers (`8 / 8` azimuth octants) |
| Near-field (`<= 15 m`) points above sensor | `2589` (`4.967 m` vertical extent) |
| Observed LiDAR scan rate (Reliable / Best Effort) | `9.90 Hz` (`133` msgs) / `2.82 Hz` (`41` msgs) |
| RealSense D435 streams (`640×480`) | Color, Infra1, Infra2, Depth (`32FC1`), `/camera/depth/color/points` (`x, y, z, rgb`) |
| Observed RealSense camera rate | `9.90 Hz` (`133` msgs) |
| Stereo `infra1` vs `infra2` | Distinct images (`infra_stereo_images_differ: true`), baseline `0.050 m` |
| Stereo `camera_depth_frame` → `camera_infra2_frame` TF | `[0.0, -0.050, 0.0]` m |
| Color `camera_depth_frame` → `camera_color_frame` TF | `[0.0, +0.015, 0.0]` m |
| Depth valid fraction / median depth | `67.7%` / `1.034 m` |
| RealSense colored cloud finite points | `49616` in `camera_depth_optical_frame` |
| Live ArUco detection at spawn (`/camera/color/image_raw`) | `ID [0]` (`start_tag_0`) |
| RViz2 `/aruco/markers` (`MarkerArray`) | `8 / 8` panels + text labels (`TRANSIENT_LOCAL`) |
| Simulation speed | `0.988×` real time |
| `base_link` → `lidar_link` translation | `[0.0, 0.0, 0.25]` m |
| Actual chassis forward movement / turn | `1.134 m` forward / `1.361 rad` turn |
| Drift after `/cmd_vel` watchdog timeout | `0.0077 m` |
| Shutdown exit codes (`logs/gazebo_shutdown.json`) | `{'server': 0, 'bridge': 0, 'state_publisher': 0, 'runtime': 0, 'gui': 0}` |

An RViz2 config smoke check (`rviz2 -d gazebo/lidar.rviz --ros-args -p use_sim_time:=true`) also loaded cleanly with zero errors.

## 3. Prior 7 m Underground Tunnel Traversal Test: PASS

Prior to enlarging the tunnel from `7 m` to `14 m`, an isolated headless drive along `y = -34.0` through the complete entry ramp, bore, and exit ramp verified drivable underground clearance and dual portal ArUco detection:

- `max_x_m`: `56.81 m` (exited `56.0 m` ramp cleanly)
- `z_inside_tunnel_min_m`: `10.837 m` (`1.483 m` below entry ground `12.320 m`)
- `min_up_cosine`: `0.963` (`final_up_cosine`: `0.999`)
- Live OpenCV `DICT_4X4_50` detections: start portal `IDs [10, 11]` and exit `IDs [20, 21]`

## 4. GUI Input & Camera Regression Test: PASS

Recorded from `.venv/bin/python gazebo/verify_controls.py` (`logs/controls_validation.json`), injecting Qt key/focus/mouse events directly into the Gazebo window (`DriveKeys` + `InputProbe`):

- `w_forward_displacement_m`: `1.440 m`
- `release_drift_m`: `0.027 m`
- `a_turn_yaw_rad`: `2.721 rad`
- `focus_out_drift_m`: `0.033 m`
- `mouse_orbit_angle_rad`: `1.103 rad` (`orbit_snapback_angle_rad`: `0.0`)
- `mouse_pan_displacement_m`: `2.396 m`
- `right_drag_zoom_displacement_m`: `9.364 m`

## Previous Stereo Configuration

The earlier stereo-only Gazebo model/configuration is archived in `backups/gazebo-stereo-20261118/`.
