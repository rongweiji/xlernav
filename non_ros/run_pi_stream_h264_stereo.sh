#!/usr/bin/env bash
set -euo pipefail

DEVICE_LEFT="/dev/video0"
DEVICE_RIGHT="/dev/video1"
WIDTH="640"
HEIGHT="480"
FPS="30"
BITRATE_KBPS="2000"
HOST=""
PORT_LEFT="5600"
PORT_RIGHT="5601"
ENCODER="auto" # auto|v4l2|omx|x264

usage() {
  cat <<USAGE
Usage: $0 --host <WSL_IP> [options]

Options:
  --device-left /dev/videoX   Left camera device (default: ${DEVICE_LEFT})
  --device-right /dev/videoX  Right camera device (default: ${DEVICE_RIGHT})
  --width N                   Frame width (default: ${WIDTH})
  --height N                  Frame height (default: ${HEIGHT})
  --fps N                     Frame rate (default: ${FPS})
  --bitrate N                 Bitrate in Kbps (default: ${BITRATE_KBPS})
  --port-left N               UDP port for left stream (default: ${PORT_LEFT})
  --port-right N              UDP port for right stream (default: ${PORT_RIGHT})
  --encoder auto|v4l2|omx|x264 Encoder selection (default: ${ENCODER})
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device-left)
      DEVICE_LEFT="${2:-}"
      shift 2
      ;;
    --device-right)
      DEVICE_RIGHT="${2:-}"
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
    --port-left)
      PORT_LEFT="${2:-}"
      shift 2
      ;;
    --port-right)
      PORT_RIGHT="${2:-}"
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

PIPELINE="\
v4l2src device=${DEVICE_LEFT} do-timestamp=true ! \
video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
queue max-size-buffers=1 leaky=downstream ! \
${PRE_ENC} ! \
${ENC} ! \
h264parse config-interval=1 ! \
rtph264pay pt=96 config-interval=1 ! \
udpsink host=${HOST} port=${PORT_LEFT} sync=false async=false \
v4l2src device=${DEVICE_RIGHT} do-timestamp=true ! \
video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
queue max-size-buffers=1 leaky=downstream ! \
${PRE_ENC} ! \
${ENC} ! \
h264parse config-interval=1 ! \
rtph264pay pt=97 config-interval=1 ! \
udpsink host=${HOST} port=${PORT_RIGHT} sync=false async=false"

echo "Starting stereo stream:"
echo "  Encoder: ${ENCODER}"
echo "  Left:    ${DEVICE_LEFT} -> udp://${HOST}:${PORT_LEFT}"
echo "  Right:   ${DEVICE_RIGHT} -> udp://${HOST}:${PORT_RIGHT}"
echo "  Size:    ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo ""

eval "gst-launch-1.0 -v ${PIPELINE}"
