



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