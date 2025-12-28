ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p camera_info_url:=file:///root/xlernav/cfg/camera_left.yaml \
  -p video_device:=/dev/video1 \
  -p brightness:=0 \
  -p contrast:=32 \
  -p saturation:=64 \
  -p hue:=0 \
  -p gamma:=100 \
  -p gain:=0 \
  -p power_line_frequency:=1 \
  -p sharpness:=3 \
  -p backlight_compensation:=12 \
  -p white_balance_automatic:=false \
  -p white_balance_temperature:=4600 \
  -r image_raw:=/image_raw \
  -r camera_info:=/camera_info \
  -p framerate:=30



QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 bash scripts/run_wsl_depth_anything_v3_viz.sh 192.168.50.124


# cehck the inference performcance 
cd /mnt/g/GithubProject/xlernav
eval "$(bash scripts/setup_cyclonedds_wsl.sh 192.168.50.124)"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run depth_anything_v3 depth_anything_v3_main --ros-args \
  --log-level depth_anything_v3:=debug \
  --params-file ros2_ws/ros2-depth-anything-v3-trt/depth_anything_v3/config/depth_anything_v3.param.yaml \
  -r '/depth_anything_v3/input/image:=/image_raw' \
  -r '/depth_anything_v3/input/camera_info:=/camera_info'


ENABLE_VIEWER=true bash scripts/run_wsl_orb_slam3_rgbd.sh 192.168.50.124