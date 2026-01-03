#  pi docker startentry1 
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
#  pi docker startentry2
 ros2 run image_transport republish --ros-args     --remap in:=/image_raw     --remap out:=/image_raw     --param in_transport:=raw     --param out_transport:=compressed

# wsl2 
# depth 
PUBLISH_POINT_CLOUD=0 bash scripts/run_wsl_depth_anything_v3_viz.sh



# wsl2 orbslam3 rgbd
USE_COMPRESSED_RGB=1 \
  COMPRESSED_RGB_TOPIC=/image_raw/compressed \
  DECOMPRESSED_RGB_TOPIC=/image_raw_uncompressed \
  bash scripts/run_wsl_orb_slam3_rgbd.sh
