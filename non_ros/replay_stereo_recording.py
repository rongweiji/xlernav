#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

import cv2


def find_latest_sample(root: Path) -> Path | None:
    candidates = [p for p in root.iterdir() if p.is_dir() and p.name.startswith("sample_")]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def build_image_map(folder: Path) -> dict[str, Path]:
    exts = {".jpg", ".jpeg", ".png"}
    result: dict[str, Path] = {}
    for entry in folder.iterdir():
        if not entry.is_file():
            continue
        if entry.suffix.lower() not in exts:
            continue
        result[entry.stem] = entry
    return result


def load_timestamp_list(path: Path) -> list[tuple[str, int]]:
    frames: list[tuple[str, int]] = []
    with path.open("r", encoding="utf-8") as handle:
        header = handle.readline()
        for line in handle:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 2:
                continue
            name, ts = parts[0].strip(), parts[1].strip()
            try:
                frames.append((name, int(ts)))
            except ValueError:
                continue
    return frames


def load_frame_list(root: Path, use_timestamps: bool) -> tuple[list[tuple[str, int | None]], dict[str, Path], dict[str, Path]]:
    left_dir = root / "left"
    right_dir = root / "right"

    if not left_dir.is_dir() or not right_dir.is_dir():
        raise RuntimeError("Missing left/ or right/ folders in recording.")

    left_map = build_image_map(left_dir)
    right_map = build_image_map(right_dir)

    if use_timestamps:
        timestamp_file = root / "timestamps.txt"
        if timestamp_file.exists():
            frames = []
            for name, ts in load_timestamp_list(timestamp_file):
                if name in left_map and name in right_map:
                    frames.append((name, ts))
            if frames:
                return frames, left_map, right_map

    # Fallback: match by intersection of filenames
    names = sorted(set(left_map.keys()) & set(right_map.keys()))
    return [(name, None) for name in names], left_map, right_map


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a recorded stereo session.")
    parser.add_argument("path", nargs="?", help="Path to sample_<timestamp> folder")
    parser.add_argument("--fps", type=float, default=None, help="Fixed playback FPS (overrides timestamps)")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default 1.0)")
    parser.add_argument("--loop", action="store_true", help="Loop playback")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor for display")
    parser.add_argument("--show-fps", action="store_true", help="Overlay playback FPS")
    parser.add_argument("--window", default="Stereo Replay", help="Window title")
    args = parser.parse_args()

    root = Path(__file__).parent
    sample_dir = Path(args.path) if args.path else None
    if sample_dir is None:
        sample_dir = find_latest_sample(root)
        if sample_dir is None:
            print("No sample_* folder found under non_ros.", file=sys.stderr)
            return 1

    if not sample_dir.exists():
        print(f"Recording folder not found: {sample_dir}", file=sys.stderr)
        return 1

    use_timestamps = args.fps is None
    frames, left_map, right_map = load_frame_list(sample_dir, use_timestamps)
    if not frames:
        print("No matching frames found.", file=sys.stderr)
        return 1

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    while True:
        base_ts = None
        base_time = None
        last_fps_time = time.perf_counter()
        fps_count = 0
        fps_value = 0.0

        for name, ts in frames:
            if use_timestamps and ts is not None:
                if base_ts is None:
                    base_ts = ts
                    base_time = time.perf_counter()
                else:
                    target = base_time + ((ts - base_ts) / 1_000_000_000.0) / max(args.speed, 1e-6)
                    delay = target - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
            elif args.fps:
                time.sleep(max(1.0 / (args.fps * max(args.speed, 1e-6)), 0.0))

            left_path = left_map.get(name)
            right_path = right_map.get(name)
            if left_path is None or right_path is None:
                continue

            left = cv2.imread(str(left_path))
            right = cv2.imread(str(right_path))
            if left is None or right is None:
                continue

            if args.scale != 1.0:
                left = cv2.resize(left, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)
                right = cv2.resize(right, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)

            combined = cv2.hconcat([left, right])

            fps_count += 1
            now = time.perf_counter()
            if now - last_fps_time >= 0.5:
                fps_value = fps_count / (now - last_fps_time)
                fps_count = 0
                last_fps_time = now

            if args.show_fps:
                cv2.putText(
                    combined,
                    f"{fps_value:.1f} fps",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow(args.window, combined)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                cv2.destroyAllWindows()
                return 0

        if not args.loop:
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
