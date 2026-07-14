# lv_precision_landing_ws

## Camera video feed to QGroundControl

The IcSpring camera is a **USB webcam** on the Jetson (`/dev/video0`), not an
IP/RTSP camera. Video is streamed to QGC over Wi-Fi as an H.264 RTP/UDP feed by
[`stream_camera_to_qgc.sh`](stream_camera_to_qgc.sh).

### Run

```bash
./stream_camera_to_qgc.sh
```

Stop with `Ctrl-C`. Edit `GCS_IP` (laptop's Wi-Fi IP) at the top of the script
if your laptop's DHCP address changes.

### QGC settings (on the laptop)

- **Application Settings → General → Video Settings**
- **Source:** `UDP h.264 Video Stream`
- **Port:** `5600`

> Note: QGC connects to the drone over the **telemetry radio** (MAVLink only).
> Video goes over **Wi-Fi**, so the laptop and Jetson must be on the same network.
> See the header comments in the script for the full list of fixes/rationale.
