# xlernav

Non-ROS visual SLAM/voxel pipelines with Rerun C++ visualization.

## What is in this repo

- Live stream path (Pi -> WSL):
  - `run_pi_stream_h264.sh`
  - `run_pi_stream_h264_stereo.sh`
  - `run_orbslam3_stream.sh` (ORB-SLAM3 + Depth Anything + Rerun)
  - `run_voxel_stream.sh` (ORB-SLAM3 + voxel map + Rerun)
- Backend selector:
  - `run_slam.sh --backend orbslam3|cuvslam`
- cuVSLAM backend source (submodule):
  - `third_party/cuVSLAM`

## Visualization

The stream apps use Rerun C++ logging instead of local OpenCV/Qt windows.

Useful flags:
- `--rerun-save <file.rrd>`
- `--no-rerun-spawn`
- `--rerun-log-every-n <N>`
- `--no-rerun-images`

## Setup

### 1) Dependencies

On Pi:
```bash
bash install_pi.sh
```

On WSL:
```bash
bash install_wsl.sh
```

### 2) Submodules

```bash
git submodule update --init --recursive
```

## Run ORB-SLAM3 stream backend

On Pi sender:
```bash
bash run_pi_stream_h264.sh --host <WSL_IP> --device /dev/video1 --encoder x264
```

On WSL receiver/tracker:
```bash
bash run_orbslam3_stream.sh --port 5600 --rerun-save outputs/orbslam3_stream.rrd
```

Voxel mapping variant:
```bash
bash run_voxel_stream.sh --port 5600 --rerun-save outputs/voxel_stream.rrd
```

## Run cuVSLAM backend

```bash
bash run_slam.sh --backend cuvslam -- \
  --dataset_root data/tum/rgbd_dataset_freiburg1_xyz \
  --dataset_format tum \
  --enable_rerun \
  --rerun_save outputs/cuvslam_tum.rrd
```

Note: `cuVSLAM` requires NVIDIA `libcuvslam.so` (see `third_party/cuVSLAM/README.md`).

## Tested dataset assets in this repo

`data/` is ignored from git and used for local testing only.

Current local dataset:
- `data/tum/rgbd_dataset_freiburg1_xyz`
- Source page: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download
- Download URL used: https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
- Calibration reference: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
- Local calibration file:
  - `data/tum/rgbd_dataset_freiburg1_xyz/camera_freiburg1_rgb.yaml`
