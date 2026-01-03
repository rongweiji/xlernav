cat > ~/.config/cyclonedds/cyclonedds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
    <Domain>
    <General><AllowMulticast>false</AllowMulticast></General>
    <Discovery><Peers><Peer address="127.0.0.1"/></Peers></Discovery>
    </Domain>
</CycloneDDS>
EOF
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/cyclonedds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p camera_info_url:=file:///root/xlernav/cfg/camera_left.yaml \
    -p video_device:=/dev/video1 \
    -r image_raw:=/image_raw \
    -r camera_info:=/camera_info \
    -p framerate:=30




export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/cyclonedds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 node list        # should show the camera node
ros2 topic list       # should show /image_raw
ros2 topic echo /image_raw --qos-profile sensor_data --once



export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/cyclonedds.xml   # include 127.0.0.1 and the Pi IP
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
source /mnt/g/GithubProject/xlernav/ros2_ws/install/setup.bash





python3 - <<'PY'
import rclpy, time
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

rclpy.init()
node = rclpy.create_node("image_fps")
qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT)
last = None
count = 0

def cb(msg):
    global last, count
    now = time.time()
    if last:
        dt = now - last
        print(f"dt={dt:.3f}s ({1/dt:.2f} fps)")
    last = now
    count += 1
    if count >= 100:
        rclpy.shutdown()

node.create_subscription(Image, "/image_raw", cb, qos)
rclpy.spin(node)
PY




unset RMW_IMPLEMENTATION
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash


ros2 daemon stop
ros2 daemon start



ros2 run image_transport republish --ros-args \
    --remap in:=/image_raw \
    --remap out:=/image_raw \
    --param in_transport:=raw \
    --param out_transport:=compressed