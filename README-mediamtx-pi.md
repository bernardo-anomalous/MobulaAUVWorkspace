MediaMTX Setup for ROV/AUV (Raspberry Pi)

Overview

This script automates the installation and configuration of MediaMTX (formerly rtsp-simple-server) on a Raspberry Pi. It is specifically optimized for headless, remote-operated vehicles (ROVs and AUVs) where reliability and resource management are critical.

Key Features

Resumable Installation: Uses a state file to remember completed steps. You can run the script multiple times without duplicating work.

Smart Architecture Detection: Automatically detects if it is running on a Raspberry Pi 4 or Pi 5 to select the optimal video encoder.

Infinite Restart Loop: The Systemd service is configured to never give up. If the camera crashes or the process dies, it restarts immediately (critical for remote operations).

Resource Optimized: Prioritizes hardware acceleration and low-latency streaming to save CPU/RAM for other processes (like ROS2).

Quick Start

Download the script to your Pi.

Make it executable:

chmod +x install_mediamtx.sh


Run with root privileges:

sudo ./install_mediamtx.sh


🎥 Encoding Modes (Automatic Selection)

The script chooses one of three encoding strategies based on your hardware and environment variables.

1. Hardware Acceleration (Default for Pi 4)

Trigger: Detected automatically on Raspberry Pi 3/4/Zero 2W.

Mechanism: Uses h264_v4l2m2m.

Benefit: Extremely low CPU usage (~5%). Uses the VideoCore GPU to compress video.

2. Software Encoding (Default for Pi 5)

Trigger: Detected automatically on Raspberry Pi 5 (which lacks H.264 HW encoder) or generic Linux PCs.

Mechanism: Uses libx264 (Preset: ultrafast, Tune: zerolatency).

Benefit: Works on any hardware. The Pi 5 CPU is powerful enough to handle this without crashing, though it consumes more power.

3. Pass-Through Mode (The "Efficiency King")

Trigger: Set environment variable CAM_ENCODES=yes.

Mechanism: Uses -vcodec copy.

Benefit: Near-zero CPU usage. Requires a USB camera that outputs H.264 natively (e.g., Logitech C920, select Arducam modules). The Pi simply forwards the packets from USB to Network without processing pixels.

Example:

sudo CAM_ENCODES=yes ./install_mediamtx.sh --force config


⚙️ Configuration Variables

You can customize the installation by setting these variables before running the script.

Variable

Default

Description

CAM_ENCODES

no

Set to yes if your camera outputs H.264. Forces copy mode.

FRAMERATE

30

Target FPS. Note: USB bandwidth may limit this (see troubleshooting).

RESOLUTION

1280x720

Video resolution (WidthxHeight).

MEDIAMTX_URL

(Auto)

Download URL. Leave empty to auto-fetch the latest ARM64 release from GitHub.

WEBRTC_HTTP_PORT

8889

Port for the WebRTC playback page.

WEBRTC_UDP_PORT

8189

UDP port for WebRTC streaming traffic.

PATH_NAME

cam

The RTSP path name (e.g., rtsp://pi:8554/cam).

STUN_SERVER

stun.l.google.com

STUN server for NAT traversal.

Example with custom settings:

sudo FRAMERATE=60 RESOLUTION=1920x1080 ./install_mediamtx.sh --force config


🛠 Usage Flags

The script tracks its progress in /var/lib/mediamtx-setup/state. If you change a setting, you must tell the script to re-run specific steps using --force.

Force a Configuration Update

If you changed variables (like resolution or encoder mode), run this to regenerate mediamtx.yml:

sudo ./install_mediamtx.sh --force config


Note: You must restart the service afterwards (sudo systemctl restart mediamtx).

Force a Full Re-Install

To wipe the previous installation state and start over:

sudo ./install_mediamtx.sh --force all


Force Service Unit Update

If you modified the Systemd logic (e.g., changing restart timers):

sudo ./install_mediamtx.sh --force unit


🐛 Troubleshooting

1. "The driver changed time per frame from 1/30 to 1/10"

Cause: USB Bandwidth saturation. Sending Raw YUYV video at 720p/30fps is too much data for the USB controller.
Fix: Force the camera to compress to MJPEG before sending data to the Pi.

Edit the script's write_conf function.

Change INPUT_FLAGS to include -input_format mjpeg.

2. Stream crashes after a few seconds

Cause: Likely OOM (Out Of Memory) or Power Undervoltage.
Check:

dmesg | grep -iE "kill|throttled|voltage"


If throttled: Improve power supply/cooling.

If kill: Switch to Hardware Encoding (Pi 4) or Pass-through mode.

3. Service keeps restarting

This is by design. The script sets Restart=always and StartLimitIntervalSec=0. The service will infinitely attempt to bring the camera back online if it disconnects (e.g., loose cable during AUV operation).

📂 File Locations

Binary: /usr/local/bin/mediamtx

Config: /etc/mediamtx/mediamtx.yml

Systemd Unit: /etc/systemd/system/mediamtx.service

Install State: /var/lib/mediamtx-setup/state