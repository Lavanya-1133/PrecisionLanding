#!/usr/bin/env bash
#
# stream_camera_to_qgc.sh
# -----------------------------------------------------------------------------
# Streams the IcSpring USB camera on the Jetson to QGroundControl as an
# H.264 RTP/UDP video feed (QGC's native "UDP h.264 Video Stream" source).
#
# Run manually on the Jetson:   ./stream_camera_to_qgc.sh
# Stop with Ctrl-C.
#
# ============================================================================
# BACKGROUND / WHAT WE FOUND (so future-you doesn't repeat the hunt)
# ============================================================================
# - The "IcSpring" camera is NOT an IP/RTSP camera. It is a plain USB webcam
#   enumerated at /dev/video0 ("icspring camera"). There is no network RTSP
#   stream to find - we generate the stream ourselves with GStreamer.
#     (A TP-Link camera at 172.29.97.141 on the shared network was a red
#      herring - someone else's device, not ours.)
#
# - QGC connects to the drone over a TELEMETRY RADIO (serial), independent of
#   the WiFi/IP network. The radio carries MAVLink only; VIDEO goes over WiFi.
#   So the laptop (GCS) and the Jetson must be on the SAME IP network.
#
# ============================================================================
# FIXES vs. the original gst-launch command
# ============================================================================
#   original                       ->  fix                       why
#   ----------------------------------------------------------------------
#   width=1280,height=720          ->  640x480                    camera's ONLY mode
#   (format unset)                 ->  format=YUY2                camera's only raw fmt
#   nvvidconv ! nvv4l2h264enc      ->  videoconvert ! x264enc     nvv4l2h264enc is NOT
#                                                                 installed here; x264enc
#                                                                 is (SW encode, trivial
#                                                                 load at 640x480)
#   host=<GCS_IP>                  ->  host=$GCS_IP (below)       real laptop WiFi IP
#   (none)                         ->  key-int-max=30             keyframe every ~1s so
#                                                                 QGC can join quickly
#
# ============================================================================
# QGC SETTINGS (on the laptop)
# ============================================================================
#   Application Settings -> General -> Video Settings
#     Source: UDP h.264 Video Stream
#     Port:   5600
#
# ============================================================================
# CAMERA CAPABILITIES (v4l2-ctl --list-formats-ext /dev/video0)
# ============================================================================
#   Format: YUYV (YUY2) only   |   Resolution: 640x480 only   |   up to 30 fps
# -----------------------------------------------------------------------------

set -euo pipefail

# ---- CONFIG (edit these if things change) -----------------------------------
GCS_IP="10.197.76.155"   # laptop's WiFi IP running QGC (DHCP - may change!)
PORT="5600"              # QGC default video port - leave as-is
DEVICE="/dev/video0"     # the IcSpring USB camera
WIDTH="640"              # camera only supports 640x480
HEIGHT="480"
FPS="30"
BITRATE_KBPS="2000"      # 2 Mbps. Drop to 1000 if the link is laggy.
# -----------------------------------------------------------------------------

echo "[*] Target GCS (QGC): ${GCS_IP}:${PORT}"

# Sanity: is the camera present?
if [[ ! -e "${DEVICE}" ]]; then
  echo "[!] ${DEVICE} not found. Is the IcSpring camera plugged in?" >&2
  exit 1
fi

# Sanity: can the Jetson reach the laptop? (video needs an IP path over WiFi)
if ping -c 1 -W 2 "${GCS_IP}" >/dev/null 2>&1; then
  echo "[*] ${GCS_IP} is reachable - starting stream."
else
  echo "[!] Cannot ping ${GCS_IP}. Check that the laptop is on the same WiFi"
  echo "    and that GCS_IP above is correct (laptop: ipconfig / ip addr)."
  echo "    Continuing anyway in case ICMP is blocked..."
fi

# ---- THE STREAM -------------------------------------------------------------
exec gst-launch-1.0 -e \
  v4l2src device="${DEVICE}" \
  ! "video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" \
  ! videoconvert \
  ! x264enc tune=zerolatency bitrate="${BITRATE_KBPS}" speed-preset=ultrafast key-int-max=30 \
  ! h264parse config-interval=1 \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host="${GCS_IP}" port="${PORT}" sync=false
