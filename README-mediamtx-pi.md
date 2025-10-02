# MediaMTX on Raspberry Pi (Ubuntu): USB Camera → Browser (WebRTC)

This README documents a **repeatable setup** for streaming a USB camera from a Raspberry Pi (Ubuntu) to any browser on the same LAN using **MediaMTX** (formerly rtsp-simple-server) with **WebRTC**.  
It includes:

- A step‑by‑step **manual guide**
- A **resumable installer script** (`setup_mediamtx_pi.sh`) that automates the process
- **Troubleshooting** and **diagnostics** playbook
- Notes on **latency tuning**, **power checks**, and **USB reliability**

---

## 1) Overview

- **Pipeline**: `USB camera (UVC) → FFmpeg (V4L2) → RTSP to MediaMTX → WebRTC (browser)`
- **Ports** (default):
  - HTTP Viewer (WebRTC signaling): `:8889`
  - WebRTC UDP port: `:8189`
  - Internal RTSP publish target (loopback): `rtsp://localhost:8554/<path>`
- **Where to view**: `http://<PI_IP>:8889/cam/` (replace `cam` if you use a different path)

---

## 2) Quick Start (Manual)

> Use this path to verify your environment or as a reference for what the script automates.

```bash
sudo apt update
sudo apt install -y ffmpeg v4l-utils curl tar udev

cd /tmp
curl -L -o mediamtx.tar.gz https://github.com/bluenviron/mediamtx/releases/latest/download/mediamtx_linux_arm64v8.tar.gz
tar -xzf mediamtx.tar.gz
sudo install -m 0755 mediamtx /usr/local/bin/mediamtx

sudo mkdir -p /etc/mediamtx
```

Create `/etc/mediamtx/mediamtx.yml`:

```yaml
webrtc: yes
webrtcAddress: :8889
webrtcLocalUDPAddress: :8189
webrtcAllowOrigin: '*'
webrtcICEServers:
  - stun:stun.l.google.com:19302

paths:
  cam:
    runOnInit: >
      ffmpeg -f v4l2 -framerate 15 -video_size 1280x720
      -i /dev/v4l/by-id/usb-REPLACE_ME-video-index0
      -vcodec libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p
      -f rtsp rtsp://localhost:8554/cam
    runOnInitRestart: yes
    source: publisher
```

> Replace the camera path using a stable symlink:
>
> ```bash
> ls -l /dev/v4l/by-id/
> ```

Create the systemd service:

```bash
sudo tee /etc/systemd/system/mediamtx.service >/dev/null <<'UNIT'
[Unit]
Description=MediaMTX real-time media router
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStartPre=/bin/sh -c 'udevadm settle --timeout=8 || true'
ExecStart=/usr/local/bin/mediamtx /etc/mediamtx/mediamtx.yml
Restart=on-failure
RestartSec=2
User=root
LimitNOFILE=1048576

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx
```

Open the viewer from any device on the LAN:

```
http://<PI_IP>:8889/cam/
```

---

## 3) Automated Installer (Resumable)

The installer script sets up everything and is **resumable**: if it fails mid‑way, re‑run it and it will **continue from the last successful step**. You can also force specific steps to re-run.

1. Copy the script contents into a file named `setup_mediamtx_pi.sh`.
2. Run it (with or without `sudo` — it uses `sudo` internally where needed):

```bash
bash setup_mediamtx_pi.sh
```

### 3.1 Configuration via Environment Variables

Override defaults by exporting variables before running:

```bash
# MediaMTX download (pin a specific version if desired)
export MEDIAMTX_URL="https://github.com/bluenviron/mediamtx/releases/download/v1.13.1/mediamtx_linux_arm64v8.tar.gz"

# Camera (use a stable by-id path if known)
export CAMERA_BYID=/dev/v4l/by-id/usb-Logitech_HD_Webcam_C270-video-index0

# Streaming parameters
export FRAMERATE=10
export RESOLUTION=640x360
export PATH_NAME=cam
export WEBRTC_HTTP_PORT=8889
export WEBRTC_UDP_PORT=8189
export STUN_SERVER=stun.l.google.com:19302

bash setup_mediamtx_pi.sh
```

### 3.2 Forcing/Re-doing Steps

The script maintains checkpoints at: `packages`, `install_bin`, `config`, `unit`, `enable_start`.

```bash
# Re-run everything regardless of checkpoints
bash setup_mediamtx_pi.sh --force all

# Re-run only the config step
bash setup_mediamtx_pi.sh --force config
```

### 3.3 What the Script Installs/Creates

- **Binary**: `/usr/local/bin/mediamtx`
- **Config**: `/etc/mediamtx/mediamtx.yml`
- **Service**: `/etc/systemd/system/mediamtx.service`
- **State** (checkpoints): `/var/lib/mediamtx-setup/state`

---

## 4) Latency Tuning (FFmpeg)

In the YAML `runOnInit` line:

- Lower resolution: `-video_size 640x360` (or 800×448, 960×540…)
- Lower fps: `-framerate 10`
- Keep: `-preset ultrafast -tune zerolatency -pix_fmt yuv420p`
- For webcams that output MJPEG faster, you can try `-input_format mjpeg` (if supported) to reduce USB bandwidth.

> Note: Phones sometimes add extra delay on WebRTC pages. Desktop browsers tend to show less latency.

After editing `/etc/mediamtx/mediamtx.yml`:

```bash
sudo systemctl restart mediamtx
```

---

## 5) Troubleshooting & Diagnostics

### 5.1 Confirm the camera exists

```bash
lsusb
v4l2-ctl --list-devices
ls -l /dev/video*
ls -l /dev/v4l/by-id/
```

If the camera doesn’t appear under `/dev/v4l/by-id/` or `/dev/video*`, it’s a **USB enumeration** issue.

### 5.2 MediaMTX logs in real-time

```bash
journalctl -u mediamtx -f
```

If the web page says “no stream available,” you’ll often see FFmpeg errors here (e.g., “Cannot open video device”).

### 5.3 Kernel messages while hot-plugging

In one terminal:

```bash
dmesg -w
```

Unplug → plug the webcam into a **USB2 (black) port** and watch for:

- `error -110` → timeout (cable/port/transient)
- `error -71` → protocol error (often cable/connector)
- `over-current` → port power issue  
- **No lines at all** → camera not detected electrically (cable, hub, bulkhead)

Optional: udev monitor

```bash
udevadm monitor --kernel --subsystem-match=usb
```

### 5.4 Ensure the UVC driver is present

```bash
sudo modprobe uvcvideo
lsmod | grep uvcvideo || echo "uvcvideo not loaded"
```

### 5.5 Soft-reset the USB host (Pi 4)

```bash
for d in /sys/bus/pci/drivers/xhci_hcd/*:*; do
  echo -n ${d##*/} | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind >/dev/null
  sleep 1
  echo -n ${d##*/} | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind   >/dev/null
done
```

### 5.6 Power sanity (undervoltage check)

Ubuntu-friendly firmware flag:

```bash
cat /sys/devices/platform/soc/soc:firmware/get_throttled 2>/dev/null || \
cat /sys/devices/platform/*/get_throttled 2>/dev/null
# 0x0 means no undervoltage/throttle this boot
```

Kernel log search:

```bash
journalctl -k -b -g 'Under-voltage\|voltage normalised\|throttl'
```

> **Note:** There is no “overvoltage flag.” Measure 5 V at the GPIO header under load; target ≤ 5.25 V.

---

## 6) Viewing & Recording

- Browser (WebRTC): `http://<PI_IP>:8889/cam/`
- Desktop test client (RTSP):  
  ```bash
  ffplay -rtsp_transport tcp rtsp://<PI_IP>:8554/cam
  ```
- Record the **WebRTC page** using an OS **screen recorder** if needed on mobile.
- To record RTSP at the Pi (headless):
  ```bash
  ffmpeg -rtsp_transport tcp -i rtsp://127.0.0.1:8554/cam -c copy -t 00:05:00 /tmp/cam.mkv
  ```

---

## 7) Common Failure Patterns & Fixes

| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| “No stream available” | FFmpeg failed to open `/dev/video*` | `journalctl -u mediamtx -f`, check camera path, `v4l2-ctl --list-devices` |
| Camera missing from `lsusb` | USB enumeration failure | Try USB2 port; different short cable; remove bulkhead/extension; powered hub |
| Intermittent camera discovery | Marginal cable/connector/hub | Replace cable; add powered hub; avoid long passive extensions |
| High latency in phone browser | Mobile WebRTC buffering | Lower fps/res; try desktop browser; ensure good Wi‑Fi |
| Service flaps on boot | Config permissions / symlink to `$HOME` | Keep config in `/etc/mediamtx/mediamtx.yml` (no symlinks) |

---

## 8) Uninstall / Cleanup

```bash
sudo systemctl disable --now mediamtx
sudo rm -f /etc/systemd/system/mediamtx.service
sudo systemctl daemon-reload

sudo rm -f /usr/local/bin/mediamtx
sudo rm -rf /etc/mediamtx

sudo rm -rf /var/lib/mediamtx-setup
```

---

## 9) Security Notes (MVP-friendly)

- The example config allows any origin (`webrtcAllowOrigin: '*'`) on your LAN. For wider networks, restrict this.
- The service runs as **root** for simplicity during MVP. When hardening, consider a dedicated user and udev rules for camera access.
- WebRTC is exposed only on your Pi’s LAN IP/ports. If you port-forward the Pi to the internet, configure **auth/TLS** appropriately.

---

## 10) FAQ

**Q: Can I stream multiple cameras?**  
A: Yes—add additional `paths:` entries (e.g., `cam2`, `cam3`) each with its own `runOnInit` and unique `-i /dev/v4l/by-id/...`.

**Q: Can I overlay HUD (pose/depth/heading)?**  
A: Easiest is on the **client** (browser) using WebRTC and a ROS2 bridge (e.g., Foxglove Bridge). Server-side overlays are also possible by inserting a GStreamer/FFmpeg filter, but add latency/CPU.

**Q: My camera only works on USB2, not USB3**  
A: Many UVC cams enumerate more reliably on Pi USB2 (black ports). Try that first.

---

## 11) Files & Paths (at a glance)

- Binary: `/usr/local/bin/mediamtx`
- Config: `/etc/mediamtx/mediamtx.yml`
- Service: `/etc/systemd/system/mediamtx.service`
- Logs: `journalctl -u mediamtx -f`
- Checkpoints: `/var/lib/mediamtx-setup/state`
- Viewer: `http://<PI_IP>:8889/cam/`

---

*Maintained by the AUV team. This README reflects the working configuration we validated in the field and is designed to be copy‑pasted on fresh Raspberry Pis for fast, consistent bring‑up.*
