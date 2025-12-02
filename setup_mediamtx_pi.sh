#!/usr/bin/env bash
# Resumable MediaMTX setup (Ubuntu on Raspberry Pi) — ROV/AUV OPTIMIZED EDITION v2
set -uo pipefail

# ===== CLI =====
FORCE_STEP="${1:-}"   # usage: --force [all|packages|install_bin|config|unit|enable_start]
if [[ "${FORCE_STEP}" == "--force" ]]; then
  FORCE_STEP="${2:-all}"; shift 2 || true
else
  FORCE_STEP=""
fi

# ===== Vars =====
# If you leave MEDIAMTX_URL empty, we'll resolve the latest arm64v8 asset from GitHub API.
MEDIAMTX_URL="${MEDIAMTX_URL:-}"

# Options: "yes" if your camera outputs H.264 natively (Logitech C920, Arducam, etc.)
CAM_ENCODES="${CAM_ENCODES:-no}"

FRAMERATE="${FRAMERATE:-30}"
RESOLUTION="${RESOLUTION:-1280x720}"
PATH_NAME="${PATH_NAME:-cam}"
WEBRTC_HTTP_PORT="${WEBRTC_HTTP_PORT:-8889}"
WEBRTC_UDP_PORT="${WEBRTC_UDP_PORT:-8189}"
STUN_SERVER="${STUN_SERVER:-stun.l.google.com:19302}"

MTX_BIN="/usr/local/bin/mediamtx"
MTX_ETC_DIR="/etc/mediamtx"
MTX_CONF="${MTX_ETC_DIR}/mediamtx.yml"
MTX_UNIT="/etc/systemd/system/mediamtx.service"
STATE_DIR="/var/lib/mediamtx-setup"
STATE_FILE="${STATE_DIR}/state"
STAMP="# managed-by: mediamtx-setup"

LOG_PREFIX="[mediamtx-setup]"
FAILS=0

need_root() { if [[ $EUID -ne 0 ]]; then SUDO="sudo"; else SUDO=""; fi; }
need_root
$SUDO mkdir -p "${STATE_DIR}" 2>/dev/null || true
$SUDO touch "${STATE_FILE}" 2>/dev/null || true

ts() { date +"%Y-%m-%d %H:%M:%S"; }
log()  { echo -e "$(ts) ${LOG_PREFIX} $*"; }
ok()   { echo -e "$(ts) ${LOG_PREFIX} OK  $*"; }
warn() { echo -e "$(ts) ${LOG_PREFIX} WARN $*" >&2; }
err()  { echo -e "$(ts) ${LOG_PREFIX} ERR $*" >&2; FAILS=$((FAILS+1)); }

show_cmd() { printf "%s %s %s\n" "$(ts)" "${LOG_PREFIX}" "➤ $*"; }
run() { show_cmd "$*"; bash -c "$@"; }

is_done() { grep -q "^$1\$" "${STATE_FILE}" 2>/dev/null; }
mark_done() { echo "$1" | $SUDO tee -a "${STATE_FILE}" >/dev/null; }
maybe_force() {
  local step="$1"
  if [[ -n "${FORCE_STEP}" ]] && { [[ "${FORCE_STEP}" == "all" ]] || [[ "${FORCE_STEP}" == "${step}" ]]; }; then
    warn "Forcing step: ${step} (ignoring checkpoint)"
    return 0
  fi
  return 1
}

step() { echo; log "=== $1 ==="; }

# ----- STEP: packages -----
do_packages() {
  step "packages"
  if is_done packages && ! maybe_force packages; then ok "packages already done"; return; fi
  export DEBIAN_FRONTEND=noninteractive
  if run "$SUDO apt-get update"; then
    if run "$SUDO apt-get install -y ffmpeg v4l-utils curl tar udev"; then
      ok "installed/verified packages"
      mark_done packages
    else
      err "package install failed"
    fi
  else
    err "apt update failed"
  fi
}

# ----- Resolve latest arm64 URL from GitHub API (with fallback) -----
resolve_latest_arm64_url() {
  local api="https://api.github.com/repos/bluenviron/mediamtx/releases/latest"
  local url
  url="$(curl -fsSL "$api" \
    | grep -oE 'https://github.com/bluenviron/mediamtx/releases/download/[^"]+/mediamtx[^"]*linux_arm64v8\.tar\.gz' \
    | head -n1 || true)"
  if [[ -n "$url" ]]; then
    echo "$url"
    return 0
  fi
  local tag
  tag="$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/tags \
    | grep -m1 -oE '"name":\s*"v[0-9]+\.[0-9]+\.[0-9]+"' \
    | head -n1 | sed 's/.*"v/v/;s/"$//' || true)"
  if [[ -n "$tag" ]]; then
    echo "https://github.com/bluenviron/mediamtx/releases/download/${tag}/mediamtx_${tag}_linux_arm64v8.tar.gz"
    return 0
  fi
  echo "https://github.com/bluenviron/mediamtx/releases/latest/download/mediamtx_linux_arm64v8.tar.gz"
}

# ----- STEP: install mediamtx -----
do_install_bin() {
  step "install_bin"
  if systemctl list-unit-files | grep -q '^mediamtx\.service'; then
    run "$SUDO systemctl stop mediamtx || true"
  fi
  if pgrep -x mediamtx >/dev/null 2>&1; then
    warn "Killing stray mediamtx process"
    run "$SUDO pkill -x mediamtx || true"
    sleep 1
  fi

  if is_done install_bin && [[ -x "${MTX_BIN}" ]] && ! maybe_force install_bin; then
    ok "binary already installed at ${MTX_BIN}"
    return
  fi

  local url
  if [[ -n "${MEDIAMTX_URL}" ]]; then
    url="${MEDIAMTX_URL}"
    log "Using MEDIAMTX_URL from env: ${url}"
  else
    url="$(resolve_latest_arm64_url)"
    log "Resolved latest asset URL: ${url}"
  fi

  local TMP
  TMP=$(mktemp -d) || { err "mktemp failed"; return; }
  cd "$TMP" || { err "cannot cd $TMP"; return; }

  if run "curl -L --progress-bar -o mediamtx.tar.gz \"${url}\""; then
    if ! tar -tzf mediamtx.tar.gz >/dev/null 2>&1; then
      err "downloaded file is not a valid tar.gz"
      return
    fi
    if run "tar -xzvf mediamtx.tar.gz"; then
      if [[ -x ./mediamtx ]]; then
        if run "$SUDO install -m 0755 ./mediamtx \"${MTX_BIN}\""; then
          ok "installed ${MTX_BIN}"
          mark_done install_bin
        else
          err "install failed"
        fi
      else
        err "archive did not contain 'mediamtx'"
      fi
    else
      err "extract failed"
    fi
  else
    err "download failed"
  fi
}

# ----- STEP: config -----
detect_cam() {
  local cam="${CAMERA_BYID:-}"
  if [[ -z "${cam}" ]]; then
    cam=$(ls /dev/v4l/by-id/*-video-index0 2>/dev/null | head -n1 || true)
    [[ -z "${cam}" ]] && cam="/dev/video0"
  fi
  echo "$cam"
}

write_conf() {
  local cam="$1"
  run "$SUDO install -d -m 755 -o root -g root \"${MTX_ETC_DIR}\"" || return 1
  
  if [[ -f "${MTX_CONF}" ]] && ! grep -q "${STAMP}" "${MTX_CONF}"; then
    run "$SUDO cp -n \"${MTX_CONF}\" \"${MTX_CONF}.bak.$(date +%Y%m%d%H%M%S)\"" || true
  fi

  # --- LATENCY & ENCODER LOGIC ---
  
  # 1. INPUT OPTIMIZATION
  # -fflags nobuffer: Don't fill a buffer before decoding
  # -flags low_delay: Tell the decoder this is realtime
  # -input_format mjpeg: Solves USB bandwidth bottleneck
  local INPUT_FLAGS="-fflags nobuffer -flags low_delay -f v4l2 -input_format mjpeg -framerate ${FRAMERATE} -video_size ${RESOLUTION}"
  local OUTPUT_FLAGS=""
  
  local MODEL
  MODEL=$(cat /sys/firmware/devicetree/base/model 2>/dev/null | tr -d '\0' || echo "Unknown")

  if [[ "${CAM_ENCODES}" == "yes" ]] || [[ "${CAM_ENCODES}" == "true" ]]; then
     log "Mode: PASS-THROUGH (Low Latency)"
     INPUT_FLAGS="${INPUT_FLAGS} -input_format h264"
     OUTPUT_FLAGS="-vcodec copy"

  elif [[ "$MODEL" == *"Raspberry Pi 4"* ]]; then
     log "Mode: PI 4 HARDWARE ENCODE (Low Latency)"
     # -g 15: Keyframe every 0.5s (fast recovery from glitches)
     # -num_capture_buffers 3: Don't buffer frames in GPU memory
     OUTPUT_FLAGS="-vcodec h264_v4l2m2m -b:v 2M -pix_fmt yuv420p -g 15 -num_capture_buffers 3"

  else
     log "Mode: SOFTWARE ENCODE (Low Latency)"
     OUTPUT_FLAGS="-vcodec libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g 15"
  fi
  # ---------------------------

  local TMP
  TMP=$(mktemp)
  cat > "${TMP}" <<YAML
${STAMP}
webrtc: yes
webrtcAddress: :${WEBRTC_HTTP_PORT}
webrtcLocalUDPAddress: :${WEBRTC_UDP_PORT}
webrtcAllowOrigin: '*'
webrtcICEServers2:
  - url: stun:${STUN_SERVER}

paths:
  ${PATH_NAME}:
    runOnInit: >
      ffmpeg ${INPUT_FLAGS}
      -i ${cam}
      ${OUTPUT_FLAGS}
      -f rtsp rtsp://localhost:8554/${PATH_NAME}
    runOnInitRestart: yes
    source: publisher
YAML
  run "$SUDO mv \"${TMP}\" \"${MTX_CONF}\""
  run "$SUDO chown root:root \"${MTX_CONF}\""
  run "$SUDO chmod 644 \"${MTX_CONF}\""
}

do_config() {
  step "config"
  if is_done config && ! maybe_force config; then ok "config already done"; return; fi
  CAM=$(detect_cam)
  log "camera path detected: ${CAM}"
  if write_conf "${CAM}"; then
    ok "wrote ${MTX_CONF}"
    mark_done config
  else
    err "failed writing config"
  fi
}

# ----- STEP: systemd unit -----
write_unit() {
  local TMP
  TMP=$(mktemp)
  cat > "${TMP}" <<UNIT
${STAMP}
[Unit]
Description=MediaMTX real-time media router
After=network-online.target
Wants=network-online.target
# CRITICAL FOR ROBOTS: Never give up restarting, even if it fails 100 times.
StartLimitIntervalSec=0

[Service]
Type=simple
ExecStartPre=/bin/sh -c 'udevadm settle --timeout=8 || true'
ExecStart=${MTX_BIN} ${MTX_CONF}
# Restart always, even if the process exits cleanly
Restart=always
RestartSec=3
User=root
LimitNOFILE=1048576

[Install]
WantedBy=multi-user.target
UNIT
  run "$SUDO mv \"${TMP}\" \"${MTX_UNIT}\""
  run "$SUDO systemctl daemon-reload"
}

do_unit() {
  step "unit"
  if is_done unit && ! maybe_force unit; then ok "unit already done"; return; fi
  if write_unit; then
    ok "installed unit"
    mark_done unit
  else
    err "failed installing unit"
  fi
}

# ----- STEP: enable+start -----
do_enable_start() {
  step "enable_start"
  if is_done enable_start && ! maybe_force enable_start; then
    ok "service enable/start already done"
    return
  fi
  run "$SUDO systemctl enable mediamtx"
  if run "$SUDO systemctl restart mediamtx"; then
    sleep 2
    if $SUDO systemctl is-active --quiet mediamtx; then
      ok "service active"
      mark_done enable_start
    else
      err "service not active"
    fi
  else
    err "start/restart failed"
  fi
}

# ----- STEP: health -----
do_health() {
  step "health"
  run "$SUDO systemctl --no-pager --full status mediamtx || true"
  run "$SUDO journalctl -u mediamtx -n 30 --no-pager || true"
  if command -v curl >/dev/null 2>&1; then
    if curl -fsS "http://127.0.0.1:${WEBRTC_HTTP_PORT}/${PATH_NAME}/" >/dev/null 2>&1; then
      ok "HTTP reachable: http://<PI_IP>:${WEBRTC_HTTP_PORT}/${PATH_NAME}/"
    else
      warn "HTTP probe failed; may still be reachable from LAN"
    fi
  fi
  GET_THR=$( { cat /sys/devices/platform/soc/soc:firmware/get_throttled 2>/dev/null || cat /sys/devices/platform/*/get_throttled 2>/dev/null; } || true )
  [[ -n "${GET_THR}" ]] && log "get_throttled=${GET_THR} (0 means clean this boot)"
}

# Run steps
do_packages
do_install_bin
do_config
do_unit
do_enable_start
do_health

echo
echo "==== SUMMARY ===="
echo "Binary:   ${MTX_BIN}"
echo "Config:   ${MTX_CONF}"
echo "Mode:     $(grep -q 'copy' ${MTX_CONF} && echo 'PASS-THROUGH' || (grep -q 'v4l2m2m' ${MTX_CONF} && echo 'HARDWARE-ACCEL' || echo 'SOFTWARE-CPU'))"
echo "State:    ${STATE_FILE}"
if [[ ${FAILS} -eq 0 ]]; then
  echo "Result:   OK"
else
  echo "Result:   FAIL (${FAILS} error(s))"
fi