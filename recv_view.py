#!/usr/bin/env python3
import argparse
import time

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst


def build_pipeline(port: int, decoder: str, payload: int) -> str:
    return (
        f"udpsrc port={port} caps="
        f"\"application/x-rtp,media=video,encoding-name=H264,payload={payload}\" ! "
        "rtph264depay ! h264parse ! "
        f"{decoder} ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink max-buffers=1 drop=true sync=false"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Receive H.264 RTP/UDP stream and show preview.")
    parser.add_argument("--port", type=int, default=5600, help="UDP port to listen on")
    parser.add_argument("--payload", type=int, default=96, help="RTP payload type (default 96)")
    parser.add_argument("--decoder", default="avdec_h264", help="GStreamer decoder element")
    parser.add_argument("--window", default="Pi Stream", help="OpenCV window title")
    parser.add_argument("--show-fps", action="store_true", help="Overlay FPS in the preview")
    parser.add_argument("--log-fps", action="store_true", help="Print FPS to stdout")
    parser.add_argument("--fps-interval", type=float, default=1.0, help="Seconds between FPS updates")
    parser.add_argument("--pull-timeout-ms", type=int, default=1000, help="Appsink pull timeout in ms")
    parser.add_argument("--log-wait", action="store_true", help="Log when no frames arrive")
    parser.add_argument("--wait-log-interval", type=float, default=5.0, help="Seconds between wait logs")
    args = parser.parse_args()

    Gst.init(None)

    pipeline_str = build_pipeline(args.port, args.decoder, args.payload)
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name("sink")

    pipeline.set_state(Gst.State.PLAYING)

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    need_fps = args.show_fps or args.log_fps
    last_fps_time = time.time()
    frame_count = 0
    fps_text = ""
    last_wait_log = 0.0
    timeout_ns = int(max(args.pull_timeout_ms, 10) * 1_000_000)

    try:
        while True:
            sample = appsink.emit("try-pull-sample", timeout_ns)
            if sample is None:
                if args.log_wait:
                    now = time.time()
                    if now - last_wait_log >= max(args.wait_log_interval, 0.1):
                        print("[stream] waiting for frames...")
                        last_wait_log = now
                continue

            buf = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")

            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                continue

            frame = np.frombuffer(map_info.data, dtype=np.uint8)
            frame = frame.reshape((height, width, 3)).copy()
            buf.unmap(map_info)

            if need_fps:
                frame_count += 1
                now = time.time()
                if now - last_fps_time >= max(args.fps_interval, 0.1):
                    fps = frame_count / (now - last_fps_time)
                    if args.log_fps:
                        print(f"[fps] {fps:.1f}")
                    if args.show_fps:
                        fps_text = f"{fps:.1f} fps"
                    frame_count = 0
                    last_fps_time = now
                if args.show_fps and fps_text:
                    cv2.putText(
                        frame,
                        fps_text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

            cv2.imshow(args.window, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
