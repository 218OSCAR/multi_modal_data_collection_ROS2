from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- configurable parameters ---
        DeclareLaunchArgument('out_dir', default_value='/home/fan/tailai_ws/src/multi_modal_data_collection/data'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('slop_sec', default_value='0.1'),
        DeclareLaunchArgument('enable_rgb', default_value='true'),
        DeclareLaunchArgument('enable_rgb2', default_value='false'),
        DeclareLaunchArgument('enable_vive', default_value='false'),
        DeclareLaunchArgument('enable_tactile', default_value='true'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera_up/color/image_raw/compressed'),
        DeclareLaunchArgument('rgb2_topic', default_value='/camera_down/color/image_raw/compressed'),
        DeclareLaunchArgument('tactile_topic', default_value='/gelsight/image_raw'),
        DeclareLaunchArgument('vive_topic', default_value='/vive_tracker/pose'),
        DeclareLaunchArgument('enable_ft', default_value='true'),
        DeclareLaunchArgument('ft_topic', default_value='/inertial_wrench_compensation/wrench_compensated'),

        # ----- Manus glove-related arguments -----
        DeclareLaunchArgument('enable_fingertip_left', default_value='false'),
        DeclareLaunchArgument('enable_fingertip_right', default_value='false'),
        DeclareLaunchArgument('enable_glove_left', default_value='false'),
        DeclareLaunchArgument('enable_glove_right', default_value='false'),

        DeclareLaunchArgument('fingertip_left_topic', default_value='/manus_fingertip_left'),
        DeclareLaunchArgument('fingertip_right_topic', default_value='/manus_fingertip_right'),
        DeclareLaunchArgument('glove_left_topic', default_value='/manus_glove_data_left'),
        DeclareLaunchArgument('glove_right_topic', default_value='/manus_glove_data_right'),

        # --- node launch ---

        # --- node launch ---
        Node(
            package='multi_modal_data_collection',
            executable='multi_sensor_data_collection',
            name='multi_sensor_data_collection',
            output='screen',
            parameters=[
                {
                    'out_dir': LaunchConfiguration('out_dir'),
                    'rate_hz': LaunchConfiguration('rate_hz'),
                    'slop_sec': LaunchConfiguration('slop_sec'),
                    'enable_rgb': LaunchConfiguration('enable_rgb'),
                    'enable_rgb2': LaunchConfiguration('enable_rgb2'),
                    'enable_vive': LaunchConfiguration('enable_vive'),
                    'enable_tactile': LaunchConfiguration('enable_tactile'),
                    'rgb_topic': LaunchConfiguration('rgb_topic'),
                    'rgb2_topic': LaunchConfiguration('rgb2_topic'),
                    'tactile_topic': LaunchConfiguration('tactile_topic'),
                    'vive_topic': LaunchConfiguration('vive_topic'),
                    'enable_ft': LaunchConfiguration('enable_ft'),
                    'ft_topic': LaunchConfiguration('ft_topic'),
                    # Manus params
                    'enable_fingertip_left': LaunchConfiguration('enable_fingertip_left'),
                    'enable_fingertip_right': LaunchConfiguration('enable_fingertip_right'),
                    'enable_glove_left': LaunchConfiguration('enable_glove_left'),
                    'enable_glove_right': LaunchConfiguration('enable_glove_right'),
                    


                    'fingertip_left_topic': LaunchConfiguration('fingertip_left_topic'),
                    'fingertip_right_topic': LaunchConfiguration('fingertip_right_topic'),
                    'glove_left_topic': LaunchConfiguration('glove_left_topic'),
                    'glove_right_topic': LaunchConfiguration('glove_right_topic'),
                }
            ]
        )
    ])
