from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # --- configurable parameters ---
        DeclareLaunchArgument(
            'out_dir', default_value='/home/fan/tailai_ws/src/multi_modal_data_collection/data'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('slop_sec', default_value='0.1'),
        DeclareLaunchArgument('enable_rgb', default_value='false'),
        DeclareLaunchArgument('enable_rgb2', default_value='true'),
        DeclareLaunchArgument('enable_rgb3', default_value='true'),
        DeclareLaunchArgument('enable_vive', default_value='false'),
        DeclareLaunchArgument('enable_tactile', default_value='true'),
        DeclareLaunchArgument(
            'rgb_topic', default_value='/camera_third_view/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'rgb2_topic', default_value='/camera_wrist_right/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'rgb3_topic', default_value='/camera_wrist_left/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'tactile_topic', default_value='/gelsight/image_raw'),
        DeclareLaunchArgument(
            'vive_topic', default_value='/vive_tracker/pose'),
        DeclareLaunchArgument('enable_ft', default_value='true'),
        DeclareLaunchArgument(
            'ft_topic', default_value='/inertial_wrench_compensation/wrench_compensated'),


        # ----- Manus glove-related arguments -----
        DeclareLaunchArgument('enable_fingertip_left', default_value='false'),
        DeclareLaunchArgument('enable_fingertip_right', default_value='false'),
        DeclareLaunchArgument('enable_glove_left', default_value='false'),
        DeclareLaunchArgument('enable_glove_right', default_value='false'),

        DeclareLaunchArgument('fingertip_left_topic',
                              default_value='/manus_fingertip_left'),
        DeclareLaunchArgument('fingertip_right_topic',
                              default_value='/manus_fingertip_right'),
        DeclareLaunchArgument('glove_left_topic',
                              default_value='/manus_glove_data_left'),
        DeclareLaunchArgument('glove_right_topic',
                              default_value='/manus_glove_data_right'),

        # ----- Vicon TCP-related arguments -----
        DeclareLaunchArgument('enable_vicon_tcp_right', default_value='true'),
        DeclareLaunchArgument('enable_vicon_tcp_left', default_value='true'),
        DeclareLaunchArgument('vicon_tcp_right_topic',
                              default_value='/tcp_pose_right'),
        DeclareLaunchArgument('vicon_tcp_left_topic',
                              default_value='/tcp_pose_left'),
        
        
        
        # ----- Fingertip width (scalar) arguments -----
        DeclareLaunchArgument('enable_fingertip_width_left', default_value='true'),
        DeclareLaunchArgument('enable_fingertip_width_right', default_value='true'),

        DeclareLaunchArgument('fingertip_width_left_topic', default_value='/fingertip_width_left'),
        DeclareLaunchArgument('fingertip_width_right_topic', default_value='/fingertip_width_right'),

        # ----- Kinect RGB arguments -----
        DeclareLaunchArgument('enable_kinect_rgb', default_value='false'),
        DeclareLaunchArgument('kinect_rgb_topic', default_value='/ak/rgb/image_raw/compressed'),

        # --- NEW node launch (timestamps version) ---
        Node(
            package='multi_modal_data_collection',
            executable='multi_sensor_data_collection_with_timestamps',
            name='multi_sensor_data_collection_with_timestamps',
            output='screen',
            parameters=[
                {
                    'out_dir': LaunchConfiguration('out_dir'),
                    'rate_hz': LaunchConfiguration('rate_hz'),
                    'slop_sec': LaunchConfiguration('slop_sec'),
                    
                    'enable_rgb': LaunchConfiguration('enable_rgb'),
                    'enable_rgb2': LaunchConfiguration('enable_rgb2'),
                    'enable_rgb3': LaunchConfiguration('enable_rgb3'),
                    'enable_vive': LaunchConfiguration('enable_vive'),
                    'enable_tactile': LaunchConfiguration('enable_tactile'),
                    'rgb_topic': LaunchConfiguration('rgb_topic'),
                    'rgb2_topic': LaunchConfiguration('rgb2_topic'),
                    'rgb3_topic': LaunchConfiguration('rgb3_topic'),
                    'tactile_topic': LaunchConfiguration('tactile_topic'),
                    'vive_topic': LaunchConfiguration('vive_topic'),
                    
                    # Manus params
                    'enable_fingertip_left': LaunchConfiguration('enable_fingertip_left'),
                    'enable_fingertip_right': LaunchConfiguration('enable_fingertip_right'),
                    'enable_glove_left': LaunchConfiguration('enable_glove_left'),
                    'enable_glove_right': LaunchConfiguration('enable_glove_right'),

                    'fingertip_left_topic': LaunchConfiguration('fingertip_left_topic'),
                    'fingertip_right_topic': LaunchConfiguration('fingertip_right_topic'),
                    'glove_left_topic': LaunchConfiguration('glove_left_topic'),
                    'glove_right_topic': LaunchConfiguration('glove_right_topic'),

                    'enable_ft': LaunchConfiguration('enable_ft'),
                    'ft_topic': LaunchConfiguration('ft_topic'),

                    # --- Vicon TCP params ---
                    'enable_vicon_tcp_right': LaunchConfiguration('enable_vicon_tcp_right'),
                    'enable_vicon_tcp_left': LaunchConfiguration('enable_vicon_tcp_left'),
                    'vicon_tcp_right_topic': LaunchConfiguration('vicon_tcp_right_topic'),
                    'vicon_tcp_left_topic': LaunchConfiguration('vicon_tcp_left_topic'),
                    
                    # --- Fingertip width params ---
                    'enable_fingertip_width_left': LaunchConfiguration('enable_fingertip_width_left'),
                    'enable_fingertip_width_right': LaunchConfiguration('enable_fingertip_width_right'),
                    'fingertip_width_left_topic': LaunchConfiguration('fingertip_width_left_topic'),
                    'fingertip_width_right_topic': LaunchConfiguration('fingertip_width_right_topic'),
                    
                    # Kinect RGB
                    'enable_kinect_rgb': LaunchConfiguration('enable_kinect_rgb'),
                    'kinect_rgb_topic': LaunchConfiguration('kinect_rgb_topic'),

                }
            ]
        )
    ])
