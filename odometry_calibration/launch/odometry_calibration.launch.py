import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_file = os.path.join(get_package_share_directory('odometry_calibration'), 'config', 'c920.yaml')
    apriltag_param_file = os.path.join(get_package_share_directory('odometry_calibration'), 'config', 'apriltags_36h11.yaml')
#    camera_info_url = 'file://' + os.path.join(get_package_share_directory('odometry_calibration'), 'config', 'c920_camera_info_1280x720.yaml')
    camera_info_url = 'file://' + os.path.join(get_package_share_directory('odometry_calibration'), 'config', 'c920_camera_info_1920x1080.yaml')

    cam_node = Node( 
        package='usb_cam', 
        executable='usb_cam_node_exe', 
        name='usb_cam', 
        parameters=[
            param_file,
            {'camera_info_url': camera_info_url}
            ], 
        output='screen',
        remappings=[ 
            ('image_raw', '/c920_camera/image_raw'),
            ('image_raw/compressed', '/c920_camera/image_raw/compressed'),
            ('image_raw/compressedDepth', '/c920_camera/image_raw/compressedDepth'),
            ('image_raw/theora', '/c920_camera/image_raw/theora'),
            ('camera_info', '/c920_camera/camera_info')
              ] )
    
    april_tag_node = Node(
        package='apriltag_ros',
        # executable='apriltag_ros_continuous_detector',
        # name='apriltag_ros_continuous_detector',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[
            apriltag_param_file,
            ], 
        output='screen',
        remappings=[ 
            ('image_rect', '/c920_camera/image_raw'),
            ('camera_info', '/c920_camera/camera_info')
              ] )

    return LaunchDescription([
        cam_node,
        april_tag_node
    ])

    # return LaunchDescription([
    #     Node(
    #         package='odometry_calibration',
    #         executable='odometry_calibration_node',
    #         name='odometry_calibration_node',
    #         output='screen'
    #     )
    # ])