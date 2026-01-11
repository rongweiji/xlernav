# Optimization Log

Track pipeline changes, refactors, and performance snapshots. Start with the
current non-ROS voxel pipeline and extend over time.

## Current Pipeline (Non-ROS, ends at voxel map)
Entry point: `non_ros/run_voxel_stream.sh` -> `non_ros/voxel_stream/main.cpp`

1) Pi sender
   - H.264 RTP/UDP stream from `/dev/videoX` (`non_ros/run_pi_stream_h264.sh`).
2) WSL receiver
   - GStreamer decode to BGR (`non_ros/common/stream_receiver.cpp`).
3) Undistort
   - `BuildUndistortMaps` + `cv::remap`.
4) Depth
   - Depth Anything V3 TensorRT (`non_ros/common/depth_estimator.hpp`).
5) Tracking
   - ORB-SLAM3 RGB-D tracking (`System::TrackRGBD`), viewer disabled.
6) Voxel integration
   - Ray-march depth into a fixed-size local grid (`VoxelMap`) in
     `non_ros/voxel_stream/voxel_map.cpp` (default 5m x 5m x 3m at 0.1m voxels).
7) Snapshot for UI
   - `VoxelMap::Snapshot` (downsampled by `--max-render`).
8) UI
   - Qt window: RGB panel + depth panel + 3D voxel + pose path view.

Threading model:
- Pipeline stages run sequentially in a single worker thread (decode -> undistort -> depth -> SLAM -> integrate -> snapshot).
- UI runs on the main Qt thread; ORB-SLAM3 may spawn internal threads, but tracking is still the main gate.

## Performance Snapshot (Baseline)
Command:
`bash non_ros/run_voxel_stream.sh --port 5600 --log-timing`

Timing (ms per stage, averaged over the log interval):
`[timing] fps=18.2 read=9.4 remap=1.2 depth=16.8 slam=24.5 integ=5.1 snap=6.7 img=3.2 loop=70.4 (frames=18 img=6 map=3 slam=18 integ=12)`

Notes:
- `fps` is end-to-end processed frames per second.
- Each timing value is milliseconds per stage.
