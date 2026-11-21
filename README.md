# Neo Rover Setup (Leo Rover + Gazebo Jetty + ROS 2)

A ready-to-run **Leo Rover** simulation in **Gazebo Jetty 10.5 (`gz-sim 10`)** and **ROS 2**, built on real 2 km Mars Desert Research Station (MDRS) terrain with rough procedural terrain & rockfields, a **32-channel 3D LiDAR**, a front **Intel RealSense D435** stereo/RGB-D camera, a Start Station, a drivable **14 m underground tunnel**, and YAML-configurable **ArUco markers**.

Clone it, run `./setup.sh` once, launch `./start.sh`, and publish standard `geometry_msgs/msg/Twist` messages to `/cmd_vel` to drive the rover.

---

## 1. Prerequisites

Tested on **Ubuntu 26.04 LTS** with **ROS 2 Lyrical** and **Gazebo Jetty 10.5** (`ROS_DISTRO=lyrical` by default; set `ROS_DISTRO=<distro>` if using another compatible release).

Install the required ROS 2, Gazebo, Qt6, and Python system packages:

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO:-lyrical}-leo-simulator \
  ros-${ROS_DISTRO:-lyrical}-leo-description \
  ros-${ROS_DISTRO:-lyrical}-leo-gz-plugins \
  ros-${ROS_DISTRO:-lyrical}-ros-gz \
  ros-${ROS_DISTRO:-lyrical}-ros-gz-bridge \
  ros-${ROS_DISTRO:-lyrical}-robot-state-publisher \
  ros-${ROS_DISTRO:-lyrical}-xacro \
  ros-${ROS_DISTRO:-lyrical}-rviz2 \
  python3-venv \
  python3-opencv \
  python3-numpy \
  python3-pil \
  python3-yaml \
  python3-requests \
  cmake \
  build-essential \
  qt6-base-dev \
  qt6-declarative-dev
```

---

## 2. Quick Setup & Launch

### Step 1: Clone and set up the world & plugins

```bash
git clone https://github.com/istiaqueahmedarik/neo_rover_setup.git
cd neo_rover_setup
./setup.sh
```

What `./setup.sh` does (works offline out of the box using the tracked `assets/terrain.npy`):
1. Verifies all required ROS 2 packages and `cv2.aruco` (`python3-opencv`).
2. Creates `.venv` (`--system-site-packages`) and installs lightweight Python dependencies from `requirements-gazebo.txt`.
3. Builds the Gazebo world (`gazebo/mdrs.sdf`), heightmap, 3-STL rockfield, underground tunnel, ArUco textures, and expanded URDF (`gazebo/leo_lidar.urdf`) via `gazebo/build_world.py`, and validates the SDF with `gz sdf -k`.
4. Compiles the `DriveKeys` / `InputProbe` Gazebo GUI plugins in `gazebo/drive_keys/`.

### Step 2: Start the simulation

```bash
./start.sh
```

This single script launches the Gazebo physics/sensor server, Gazebo 3D viewer, `ros_gz_bridge`, `robot_state_publisher`, and the command watchdog + stereo/ArUco runtime (`gazebo/runtime.py`).

Optional flags:

```bash
./start.sh --headless      # physics + sensors + ROS bridges only (no GUI)
./start.sh --duration 30   # exit automatically after 30 wall seconds
```

Press `Ctrl+C` in the launch terminal or close the Gazebo window to stop all child processes cleanly.

---

## 3. Moving the Rover via `/cmd_vel`

Once `./start.sh` is running, any ROS 2 node or CLI command on your machine can drive the rover by publishing `geometry_msgs/msg/Twist` messages to **`/cmd_vel`**.

### Command-line examples

Open a second terminal inside `neo_rover_setup` and source the environment:

```bash
cd neo_rover_setup
source ./ros_env.sh
ros2 topic list
```

Drive forward at `0.4 m/s`:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Reverse at `-0.3 m/s`:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.3}, angular: {z: 0.0}}'
```

Turn in place left (`+z`) or right (`-z`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.6}}'
```

Drive forward while steering in a curve:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.4}, angular: {z: 0.3}}'
```

### Python (`rclpy`) `/cmd_vel` example

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)  # >= 5 Hz keeps watchdog active

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.4   # m/s  (clamped to [-0.8, 0.8])
        msg.angular.z = 0.2  # rad/s (clamped to [-1.5, 1.5])
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RoverDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### `/cmd_vel` behavior & safety watchdog

- **Message type**: `geometry_msgs/msg/Twist` (not `TwistStamped`). Only `linear.x` and `angular.z` are used (`+x` forward, `+z` counterclockwise / turn left).
- **Velocity limits**: Clamped by [runtime.py](gazebo/runtime.py) and Gazebo `DiffDrive` to **`±0.8 m/s`** linear and **`±1.5 rad/s`** angular, with wheel acceleration limits.
- **0.5 s safety watchdog**: You must publish continuously (e.g., 10 Hz with `-r 10` or a timer) to keep moving. If no `/cmd_vel` message arrives for **0.5 s** (for instance when you hit `Ctrl+C`), [runtime.py](gazebo/runtime.py) automatically sends zero velocity so the rover brakes cleanly. Non-finite (`NaN`/`Inf`) commands are ignored.
- **Always publish to `/cmd_vel`**, not `/simulation/cmd_vel` (which is the internal watchdog-to-Gazebo topic).
- On slopes, rocks, and ramps, actual 3D motion differs slightly from planar wheel odometry (`/odom`) due to wheel slip and rocker-bogie articulation — use `/simulation/ground_truth` to inspect exact world pose.

---

## 4. Manual Keyboard & Camera Controls

- **Gazebo window W/A/S/D**: Click inside the 3D scene to focus Gazebo, then hold **W** (forward), **S** (reverse), or **A/D** (turn). Releasing keys or pressing **Space** stops the rover; switching window focus also stops motion. When no key is held, `DriveKeys` stays idle so external `/cmd_vel` nodes have full control.
- **Terminal teleop**: Run `./teleop.sh` in another terminal for terminal-based `W/A/S/D` control (`Space` stops, `Q` quits).
- **Free Gazebo camera**:
  - **Shift + left-drag** or **middle-drag**: orbit / look around
  - **Left-drag**: pan across the map
  - **Scroll wheel** or **right-drag**: zoom
  - Reset Follow mode back to free camera:
    ```bash
    source ./ros_env.sh
    gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: NONE'
    ```

---

## 5. Visualize in RViz2

In a second terminal:

```bash
./rviz.sh
```

[lidar.rviz](gazebo/lidar.rviz) opens RViz2 with `use_sim_time:=true`, Fixed Frame `odom`, `RobotModel`, `/lidar/points` colored by Z height (`AxisColor`), `/camera/color/image_raw`, `/aruco/markers` (`MarkerArray`), and optional Infra1 / Infra2 / Depth Cloud displays ready to toggle.

---

## 6. ROS 2 Topics & Sensors

| Topic | Type | Details |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Drive velocity input (`linear.x` ±0.8 m/s, `angular.z` ±1.5 rad/s) |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo simulation time (set `use_sim_time:=true` on ROS nodes) |
| `/odom` | `nav_msgs/msg/Odometry` | Official Leo differential-drive wheel odometry (`odom` → `base_footprint`) |
| `/simulation/ground_truth` | `nav_msgs/msg/Odometry` | Exact 3D chassis pose and velocity in world frame |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `odom` → `base_footprint` → `chassis`, rockers, wheels, IMU, LiDAR, RealSense frames |
| `/joint_states` | `sensor_msgs/msg/JointState` | 4 wheel joints + 2 rocker joints |
| `/imu/data` | `sensor_msgs/msg/Imu` | Onboard IMU (`imu_frame`, best-effort sensor QoS) |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | 32 channels × 1024 samples, 360°, `-30°..+15°` vertical, `0.2..80 m`, 10 Hz (`lidar_link`, Reliable QoS) |
| `/camera/color/image_raw`, `/camera/color/camera_info` | `Image` / `CameraInfo` | RealSense D435 RGB (`640×480`, 10 Hz, `camera_color_optical_frame`) |
| `/camera/infra1/image_raw`, `/camera/infra1/camera_info` | `Image` / `CameraInfo` | RealSense D435 left stereo IR (`640×480`, 10 Hz, `camera_depth_optical_frame`) |
| `/camera/infra2/image_raw`, `/camera/infra2/camera_info` | `Image` / `CameraInfo` | RealSense D435 right stereo IR (`640×480`, 10 Hz, 50 mm baseline, calibrated `P[3]`, `camera_infra2_optical_frame`) |
| `/camera/depth/image_raw`, `/camera/depth/camera_info` | `Image` / `CameraInfo` | RealSense D435 metric depth (`32FC1`, `640×480`, 10 Hz, `camera_depth_optical_frame`) |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Lazy RGB-D cloud (`x, y, z, rgb`, `320×240`, published when subscribed) |
| `/aruco/markers` | `visualization_msgs/msg/MarkerArray` | All configured ArUco panels + text labels in `odom` (`TRANSIENT_LOCAL`) |
| `/robot_description` | `std_msgs/msg/String` | Expanded Leo Rover URDF with LiDAR & RealSense D435 links |

Quick live checks:

```bash
source ./ros_env.sh
ros2 topic hz /lidar/points
ros2 topic echo /simulation/ground_truth --once
ros2 topic echo /camera/infra2/camera_info --once
```

Notice on RealSense D435: Gazebo Classic's `librealsense_gazebo_plugin.so` does not load in Gazebo Jetty (`gz-sim 10`), so the rover implements the identical `camera_link` / `camera_{depth,infra1,infra2,color}_{frame,optical_frame}` TF layout, `50 mm` baseline, `+15 mm` color offset, and `/camera/*` ROS 2 topics natively using Gazebo Jetty Ogre2 sensors, `ros_gz_bridge`, and [runtime.py](gazebo/runtime.py).

Both `./start.sh` and `./ros_env.sh` set `GZ_PARTITION=my_rover_$UID`. Keep the same `ROS_DOMAIN_ID` (default `0`) across terminals.

---

## 7. Configurable ArUco Markers (`gazebo/aruco_markers.yaml`)

All ArUco markers in Gazebo (rendered as OpenCV-generated textures on UV-mapped meshes) and RViz2 (`/aruco/markers`) are defined in [aruco_markers.yaml](gazebo/aruco_markers.yaml):

```yaml
defaults:
  dictionary: DICT_4X4_50
  size_cm: [20.0, 20.0]   # variable per marker: scalar (e.g. 25.0) or [width_cm, height_cm]
  border_bits: 1
  white_margin_ratio: 0.15

markers:
  - name: tunnel_start_left
    id: 10
    dictionary: DICT_4X4_50
    size_cm: [20.0, 20.0]
    position:
      x: 36.665
      y: -31.80
      z_mode: tunnel_entry   # supports ground (`z_above_ground`), `z`, `tunnel_floor`, `tunnel_entry`, `tunnel_exit`
      z_offset: 0.48
    yaw_deg: 180.0
    with_post: false
    double_sided: false
    role: tunnel_start
```

After editing `gazebo/aruco_markers.yaml`, regenerate the textures and world:

```bash
source ./ros_env.sh
.venv/bin/python gazebo/build_world.py
```

---

## 8. Environment, Start Station & Underground Tunnel

- **Terrain**: 2,000 × 2,000 m bare-earth DEM cropped near MDRS, Utah (`-110.7919, 38.4073`) from USGS 3DEP QL1 data (`assets/terrain.npy`), augmented with multi-octave roughness, craters, ridges, and ~420 jagged rocks in three static STL meshes. If `assets/terrain.npy` is removed, [build_terrain.py](scripts/build_terrain.py) downloads and re-crops it from USGS.
- **Rover spawn & Start Station**: Rover spawns on a flat pad at `(30.0, -34.0)` facing `+x` east with `start_tag_0` (`ID 0`) ahead; a Start Station outpost (`start_station`) stands at `(34.5, -26.5)` with habitat dome, solar array, comms mast, and crates.
- **14 m Underground Tunnel** (along `y = -34.0` directly ahead of spawn):
  - **Entry ramp**: `x = 37.0 → 47.0 m` (`10.0 m` ramp descending `1.80 m` below exit ground, flanked by retaining sidebars and vertical entrance pillars)
  - **Underground bore**: `x = 47.0 → 61.0 m` (`14.0 m` long, `3.40 m` clear width, `2.20 m` headroom, buried under an earthen overburden hill with interior point lights)
  - **Exit ramp**: `x = 61.0 → 71.0 m` (`10.0 m` ramp climbing back out to the desert floor)
  - **Tunnel ArUco tags**:
    - Front start pair: `ID 10` (`y = -31.80`) & `ID 11` (`y = -36.20`) mounted **in front of the two entrance sidebars** at `x = 36.665` (`DICT_4X4_50`, `20×20 cm`)
    - Rear end pair: `ID 20` (`y = -32.45`) & `ID 21` (`y = -35.55`) at the bore exit `x = 61.90` (`DICT_4X4_50`, `20×20 cm`, double-sided)

---

## 9. Repository Layout

```text
neo_rover_setup/
├── setup.sh                  # One-time setup: checks ROS/OpenCV, venv, builds world & DriveKeys plugin
├── start.sh                  # Launches Gazebo server + GUI + bridge + TF + runtime watchdog
├── rviz.sh                   # Launches RViz2 pre-configured for 3D LiDAR, RealSense & ArUco markers
├── teleop.sh / scripts/teleop.py  # Optional terminal W/A/S/D /cmd_vel teleop
├── ros_env.sh                # Sources ROS 2 setup.bash and sets GZ_PARTITION
├── requirements-gazebo.txt   # Python packages installed inside .venv
├── VALIDATION.md             # Recorded sensor, physics, tunnel & GUI control validation results
├── assets/
│   ├── terrain.npy           # Pre-cropped 2 km MDRS elevation grid (1001x1001)
│   ├── terrain.json          # DEM metadata & geographic coordinates
│   └── usgs_catalog.json     # Upstream USGS 3DEP source metadata
├── scripts/
│   ├── build_terrain.py      # Re-downloads & crops USGS GeoTIFF if assets/terrain.npy is missing
│   └── resume_download.py    # Resumable HTTP range downloader for USGS GeoTIFF
└── gazebo/
    ├── aruco_markers.yaml    # Configurable list of ArUco IDs, dictionaries, positions & sizes
    ├── build_world.py        # Generates mdrs.sdf, heightmap, rocks, tunnel & ArUco textures
    ├── bridge.yaml           # ros_gz_bridge topic mappings
    ├── gui.config            # Gazebo Jetty GUI layout & DriveKeys plugin config
    ├── lidar.rviz            # Pre-configured RViz2 layout
    ├── runtime.py            # /cmd_vel watchdog, D435 stereo CameraInfo/RGB-D cloud, /aruco/markers
    ├── supervisor.py         # Process supervisor for server, bridge, state_publisher, runtime, GUI
    ├── verify_gazebo.py      # End-to-end ROS + Gazebo integration test (--gui or headless)
    ├── verify_controls.py    # Qt input regression test for Gazebo keyboard & camera controls
    └── drive_keys/           # C++ Qt/Gazebo plugins (DriveKeys + InputProbe test plugin)
```

---

## 10. Verification & Troubleshooting

To run the automated integration checks (stop any running `./start.sh` instance first):

```bash
.venv/bin/python gazebo/verify_gazebo.py --gui
.venv/bin/python gazebo/verify_controls.py
```

Common issues:
- **"This rover simulation is already running"**: Only one instance per checkout can hold `.simulation.lock`. Close your existing `./start.sh` session first.
- **Rover doesn't move when publishing `/cmd_vel`**: Verify you are publishing `geometry_msgs/msg/Twist` to `/cmd_vel` at ≥ 5 Hz (`-r 10`), `./ros_env.sh` is sourced, and `ROS_DOMAIN_ID` matches across terminals.
- **RViz shows no LiDAR / camera data or wrong timestamps**: Always launch with `./rviz.sh` (or pass `--ros-args -p use_sim_time:=true`) so RViz subscribes with `Reliable` QoS and simulation time.
- **Non-NVIDIA GPUs / Wayland**: `start.sh` only enables `__NV_PRIME_RENDER_OFFLOAD` when `nvidia-smi` is detected. If your display server requires native Wayland instead of XWayland, run `QT_QPA_PLATFORM=wayland ./start.sh`.
