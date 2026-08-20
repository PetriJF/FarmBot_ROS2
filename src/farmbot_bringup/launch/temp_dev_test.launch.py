"""Temporary dev launch: the full stack task_sequencer's commands (P_R, P_S, ...) depend on."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description with only the refactored nodes."""
    ws_path = LaunchConfiguration('ws_path')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_path',
            default_value=os.path.expanduser('~/FarmBot_ROS2/farmbot_data'),
            description='Path to the farmbot_data folder, which contains the config files.'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for communicating with the FarmBot.'
        ),

        # The map handler (map_info, MapSize, plant list)
        Node(
            package='map_handler',
            executable='map_controller',
            name='map_controller',
            output='screen',
            parameters=[
                {'ws_path': ws_path},
                {'folder_config_name': 'local_config'},
            ]
        ),
        # The Standard USB camera
        Node(
            package='farmbot_vision',
            executable='standard_camera',
            name='standard_camera',
            output='screen',
            parameters=[
                {'camera_index': 0},
                {'image_width': 640},
                {'image_height': 480},
                {'frame_rate': 30.0},
                {'pixel_format': 'MJPG'},  # MJPG avoids USB-bandwidth fps throttling
                {'frame_id': 'camera'},
                {'auto_exposure': False},  # manual exposure needed for full 30 fps
                {'exposure': 156.0},       # tune for scene brightness (higher = brighter/darker)
            ]
        ),
        # Plant radius measurement (P_R's vision dependency)
        Node(
            package='farmbot_vision',
            executable='plant_radius',
            name='plant_radius',
            output='screen',
            parameters=[
                {'hsv_min': [40, 50, 50]},
                {'hsv_max': [90, 255, 255]},
                {'min_contour_area_px': 100},
                {'mm_per_pixel': 0.5},           # placeholder, needs calibration
                {'plant_radius_padding_mm': 20.0},
            ]
        ),
        # Camera calibration (camera/calibrate)
        Node(
            package='farmbot_vision',
            executable='camera_calibration',
            name='camera_calibration',
            output='screen',
            parameters=[
                {'ws_path': ws_path},
                {'folder_config_name': 'local_config'},
            ]
        ),
        # Image stitching (P_S's vision dependency)
        Node(
            package='farmbot_vision',
            executable='image_stitcher',
            name='image_stitcher',
            output='screen',
            parameters=[
                {'ws_path': ws_path},
                {'folder_config_name': 'local_config'},
                {'map_mm_per_px': 0.5},          # map resolution; raise to shrink the canvas
                {'feather_px': 40},              # seam softness across overlapping frames
                {'frame_crop_fraction': 0.8},    # drop the lens-distorted frame border
                {'camera_offset_x': 18.0},       # lens offset from the reported gantry position
                {'camera_offset_y': 65.0},
            ]
        ),
        # The serial bridge
        Node(
            package='farmbot_hardware_comm',
            executable='serial_controller',
            name='serial_controller',
            output='screen',
            parameters=[
                {'serial_port': serial_port},
                {'serial_speed': 115200},
                {'check_serial_freq': 100},
                {'ws_path': ws_path},
                {'folder_config_name': 'local_config'},
            ]
        ),
        # The GPIO controller
        Node(
            package='farmbot_hardware_comm',
            executable='gpio_controller',
            name='gpio_controller',
            output='screen',
            parameters=[
                {'flashing_frequency': 2.0},
            ]
        ),
        # The sequence engine
        Node(
            package='farmbot_controllers',
            executable='task_sequencer',
            name='task_sequencer',
            output='screen',
            parameters=[
                # TODO: placeholder, needs calibrating together with plant_radius's
                # mm_per_pixel (see standard.launch.py)
                {'radius_capture_z': 0.0},
                # TODO: placeholder, needs calibrating together with image_stitcher's
                # camera/calibrate output (see standard.launch.py)
                {'stitch_capture_z': 0.0},
                # ~55% overlap for the stitcher's feathering, assuming 0.5mm_per_px
                # (640x480px, 0.8 frame_crop_fraction); recompute once camera/calibrate
                # has been run for real
                {'stitch_step_mm': 135.0},
                # Must match image_stitcher's camera_offset_x/y above - same physical offset.
                {'camera_offset_x': 18.0},
                {'camera_offset_y': 65.0},
            ]
        ),
    ])
