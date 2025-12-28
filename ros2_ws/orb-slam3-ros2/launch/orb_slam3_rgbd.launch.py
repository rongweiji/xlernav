from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vocab_path', default_value='/mnt/g/LocalGitProject/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        DeclareLaunchArgument('settings_path', default_value='/mnt/g/GithubProject/xlernav/ros2_ws/orb-slam3-ros2/config/raspi_rgbd.yaml'),
        DeclareLaunchArgument('rgb_topic', default_value='/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_anything_v3/output/depth_image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('enable_viewer', default_value='false'),
        DeclareLaunchArgument('image_encoding', default_value=''),
        DeclareLaunchArgument('sync_queue_size', default_value='10'),
        DeclareLaunchArgument('sync_slop_sec', default_value='0.05'),
        DeclareLaunchArgument('diagnostics_period_sec', default_value='2.0'),
        DeclareLaunchArgument('depth_stats_stride', default_value='8'),
        Node(
            package='orb_slam3_ros2',
            executable='orb_slam3_rgbd_node',
            name='orb_slam3_rgbd',
            output='screen',
            parameters=[{
                'vocab_path': LaunchConfiguration('vocab_path'),
                'settings_path': LaunchConfiguration('settings_path'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'enable_viewer': LaunchConfiguration('enable_viewer'),
                'image_encoding': LaunchConfiguration('image_encoding'),
                'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                'sync_slop_sec': LaunchConfiguration('sync_slop_sec'),
                'diagnostics_period_sec': LaunchConfiguration('diagnostics_period_sec'),
                'depth_stats_stride': LaunchConfiguration('depth_stats_stride'),
            }],
        )
    ])
