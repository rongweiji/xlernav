#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/video0"
WIDTH="640"
HEIGHT="480"
FPS="30"
BITRATE_KBPS="2000"
HOST=""
PORT="5600"
ENCODER="auto" # auto|v4l2|omx|x264

usage() {
  cat <<EOF
Usage: $0 --host <WSL_IP> [options]

Options:
  --device /dev/videoX    Camera device (default: ${DEVICE})
  --width N               Frame width (default: ${WIDTH})
  --height N              Frame height (default: ${HEIGHT})
  --fps N                 Frame rate (default: ${FPS})
  --bitrate N             Bitrate in Kbps (default: ${BITRATE_KBPS})
  --port N                UDP port (default: ${PORT})
  --encoder auto|v4l2|omx|x264  Encoder selection (default: ${ENCODER})
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)
      DEVICE="${2:-}"
      shift 2
      ;;
    --width)
      WIDTH="${2:-}"
      shift 2
      ;;
    --height)
      HEIGHT="${2:-}"
      shift 2
      ;;
    --fps)
      FPS="${2:-}"
      shift 2
      ;;
    --bitrate)
      BITRATE_KBPS="${2:-}"
      shift 2
      ;;
    --host)
      HOST="${2:-}"
      shift 2
      ;;
    --port)
      PORT="${2:-}"
      shift 2
      ;;
    --encoder)
      ENCODER="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${HOST}" ]]; then
  echo "Error: --host <WSL_IP> is required." >&2
  usage
  exit 1
fi

if [[ "${ENCODER}" == "auto" ]]; then
  if gst-inspect-1.0 v4l2h264enc >/dev/null 2>&1; then
    ENCODER="v4l2"
  elif gst-inspect-1.0 omxh264enc >/dev/null 2>&1; then
    ENCODER="omx"
  else
    ENCODER="x264"
  fi
fi

BITRATE_BPS="$((BITRATE_KBPS * 1000))"

case "${ENCODER}" in
  v4l2)
    PRE_ENC="videoconvert ! video/x-raw,format=NV12"
    ENC="v4l2h264enc extra-controls=\"controls,video_bitrate=${BITRATE_BPS}\""
    ;;
  omx)
    PRE_ENC="videoconvert ! video/x-raw,format=I420"
    ENC="omxh264enc target-bitrate=${BITRATE_BPS} control-rate=variable"
    ;;
  x264)
    PRE_ENC="videoconvert ! video/x-raw,format=I420"
    ENC="x264enc bitrate=${BITRATE_KBPS} speed-preset=ultrafast tune=zerolatency key-int-max=${FPS}"
    ;;
  *)
    echo "Unknown encoder: ${ENCODER}" >&2
    exit 1
    ;;
esac

PIPELINE="v4l2src device=${DEVICE} ! \
video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
${PRE_ENC} ! \
${ENC} ! \
h264parse config-interval=1 ! \
rtph264pay pt=96 config-interval=1 ! \
udpsink host=${HOST} port=${PORT} sync=false async=false"

echo "Starting stream:"
echo "  Encoder: ${ENCODER}"
echo "  Device:  ${DEVICE}"
echo "  Size:    ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo "  Target:  udp://${HOST}:${PORT}"
echo ""

eval "gst-launch-1.0 -v ${PIPELINE}"
