# PrecisionLanding

Autonomous precision landing of a quadrotor UAV on static and moving ground platforms. Uses **PX4 + ROS 2 offboard control** with **ArUco vision-based relative pose estimation**, and benchmarks **PID, LQR/LQG, and MPC** controllers in **Gazebo SITL** and on hardware.

## How it works

```
USB camera ──▶ Jetson (ArUco pose, ROS 2 control) ──MAVLink LANDING_TARGET──▶ Pixhawk 6C (PX4)
                    │
                    └── Wi-Fi: H.264 RTP/UDP (port 5600) ──▶ QGroundControl
```

A downward-facing camera detects an ArUco marker on the landing pad, estimates its 6-DOF pose with `solvePnP`, and streams `LANDING_TARGET` messages to PX4 at 20 Hz. The ROS 2 package uses this relative pose for the landing controllers.

## Hardware

- Pixhawk 6C running PX4 (USB-serial, `/dev/ttyACM0`)
- NVIDIA Jetson companion computer
- Hiwonder icSpring USB camera (640x480, YUY2, 30 fps)
- ArUco marker `DICT_6X6_100`, ID 0, 15 cm
- Checkerboard (5x7 inner corners, 36.2 mm squares) for calibration

## Repository structure

| Path | Purpose |
|---|---|
| `pl_pkg/` | ROS 2 package (offboard control and landing logic) |
| `aruco_detect.py` | ArUco detection, pose estimation, MAVLink output |
| `calibrate_camera.py` | Interactive camera calibration |
| `camera_calibration.npz` | Saved calibration data |
| `test_camera.py` | Camera sanity check |
| `stream_camera_to_qgc.sh` | Video stream to QGroundControl |
| `Lavanya_Bhatnagar_Internship_Report.pdf` | Full project report |

## Requirements

- Ubuntu with ROS 2 and PX4 (Gazebo SITL for simulation)
- Python 3, GStreamer (`gst-launch-1.0` with `x264enc`)
- [QGroundControl](http://qgroundcontrol.com/) on a laptop sharing the Jetson's Wi-Fi network

```bash
pip install opencv-contrib-python pymavlink numpy
```

`aruco_detect.py` uses the legacy `cv2.aruco` API, so use an OpenCV version that still includes it (e.g. 4.6.x).

## Usage

```bash
git clone https://github.com/Lavanya-1133/PrecisionLanding.git
cd PrecisionLanding
```

**1. Calibrate the camera** (a calibration file is included; redo it for a different camera)

```bash
python calibrate_camera.py
```

Press `SPACE` to capture views of the checkerboard (20 recommended), then `Q` to calibrate and save.

**2. Test the camera**

```bash
python test_camera.py
```

**3. Detect the marker and send landing targets**

```bash
python aruco_detect.py                                   # defaults
python aruco_detect.py --port /dev/ttyUSB0 --baud 57600 --marker-id 7 --marker-size 0.20
python aruco_detect.py --no-mavlink                      # camera only, for tuning
```

Press `Q` to quit, `S` to save a snapshot.

**4. Stream video to QGroundControl**

Set `GCS_IP` in `stream_camera_to_qgc.sh` to your laptop's IP, then run:

```bash
./stream_camera_to_qgc.sh
```

In QGC: **Application Settings → General → Video Settings**, set Source to `UDP h.264 Video Stream` and Port to `5600`. (MAVLink goes over the telemetry radio; video goes over Wi-Fi.)

**5. Build the ROS 2 package**

```bash
mkdir -p ~/lv_precision_landing_ws/src
cp -r pl_pkg ~/lv_precision_landing_ws/src/
cd ~/lv_precision_landing_ws
colcon build --packages-select pl_pkg
source install/setup.bash
```

## Controllers

| Controller | Description |
|---|---|
| PID | Baseline feedback on the relative pose error |
| LQR / LQG | Optimal state feedback, with a Kalman filter for noisy vision data |
| MPC | Constrained predictive control over a finite horizon |

Each is tested on static and moving platforms. Results and analysis are in the [project report](Lavanya_Bhatnagar_Internship_Report.pdf).

## Troubleshooting

- **Camera won't open:** `aruco_detect.py` uses a hard-coded `DEVICE_INDEX`. Find yours with `v4l2-ctl --list-devices` and update it.
- **MAVLink won't connect:** check the port (`ls /dev/ttyACM*`) and baud rate, and make sure no other program holds the port.
- **No video in QGC:** confirm `GCS_IP` is current, both devices are on the same network, and QGC is on port `5600`. If laggy, lower `BITRATE_KBPS` in the script.

## Author

**Lavanya Bhatnagar** ([@Lavanya-1133](https://github.com/Lavanya-1133))
