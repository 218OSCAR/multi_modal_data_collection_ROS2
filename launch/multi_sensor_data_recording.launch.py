from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('out_dir', default_value='/home/tailai.cheng/tailai_ws/src/multi_modal_data_collection/data'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('slop_sec', default_value='0.10'),

        # enable sensors
        DeclareLaunchArgument('enable_rs', default_value='true'),
        DeclareLaunchArgument('enable_kinect', default_value='false'),
        DeclareLaunchArgument('enable_kinect2', default_value='false'),
        DeclareLaunchArgument('enable_arm', default_value='false'),
        DeclareLaunchArgument('enable_grip', default_value='false'),
        DeclareLaunchArgument('enable_base', default_value='false'),
        DeclareLaunchArgument('enable_base_pose', default_value='false'),
        DeclareLaunchArgument('enable_tactile', default_value='true'),
        DeclareLaunchArgument('enable_ft', default_value='true'),

        # depth
        DeclareLaunchArgument('enable_rs_depth', default_value='false'),
        DeclareLaunchArgument('enable_kinect_depth', default_value='false'),
        DeclareLaunchArgument('enable_kinect2_depth', default_value='false'),

        # topic parameters
        DeclareLaunchArgument('rs_color_topic', default_value='/camera/color/image_raw/compressed'),
        DeclareLaunchArgument('rs_depth_topic', default_value='/camera/aligned_depth_to_color/image_raw/compressed'),
        DeclareLaunchArgument('kinect_color_topic', default_value='/k4a1/rgb/image_raw/compressed'),
        DeclareLaunchArgument('kinect_depth_topic', default_value='/k4a1/depth_to_rgb/image_raw/compressed'),
        DeclareLaunchArgument('kinect2_color_topic', default_value='/robot/top_right_rgbd_camera_2/rgb/image_raw/compressed'),
        DeclareLaunchArgument('kinect2_depth_topic', default_value='/robot/top_right_rgbd_camera_2/depth_to_rgb/image_raw/compressed'),

        # tactile and ft
        DeclareLaunchArgument('tactile_topic', default_value='/gelsight/image_raw'),
        DeclareLaunchArgument('ft_topic', default_value='/inertial_wrench_compensation/wrench_compensated'),

        # robot state
        DeclareLaunchArgument('arm_joint_topic', default_value='/franka_state_controller/joint_states'),
        DeclareLaunchArgument('require_ee', default_value='false'),
        DeclareLaunchArgument('base_odom_topic', default_value='/robot/robotnik_base_control/odom'),

        # timestamp and prewarm
        DeclareLaunchArgument('img_stamp_mode', default_value='auto'),
        DeclareLaunchArgument('max_stamp_skew', default_value='1.0'),
        DeclareLaunchArgument('rs_prewarm_frames', default_value='30'),
        DeclareLaunchArgument('kinect_prewarm_frames', default_value='30'),
        DeclareLaunchArgument('tactile_prewarm_frames', default_value='5'),
    ]

    # Node for data recording
    data_record_node = Node(
        package='data_record',
        executable='tabletask_datarecord_1.py',  # 注意: ROS2 可执行文件必须在 setup.py entry_points 中注册
        name='darko_tabletask_datarecord_1',
        output='screen',
        parameters=[{
            'out_dir': LaunchConfiguration('out_dir'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'slop_sec': LaunchConfiguration('slop_sec'),

            'enable_rs': LaunchConfiguration('enable_rs'),
            'enable_kinect': LaunchConfiguration('enable_kinect'),
            'enable_kinect2': LaunchConfiguration('enable_kinect2'),
            'enable_arm': LaunchConfiguration('enable_arm'),
            'enable_grip': LaunchConfiguration('enable_grip'),
            'enable_base': LaunchConfiguration('enable_base'),
            'enable_base_pose': LaunchConfiguration('enable_base_pose'),
            'enable_tactile': LaunchConfiguration('enable_tactile'),
            'enable_ft': LaunchConfiguration('enable_ft'),

            'enable_rs_depth': LaunchConfiguration('enable_rs_depth'),
            'enable_kinect_depth': LaunchConfiguration('enable_kinect_depth'),
            'enable_kinect2_depth': LaunchConfiguration('enable_kinect2_depth'),

            'rs_color_topic': LaunchConfiguration('rs_color_topic'),
            'rs_depth_topic': LaunchConfiguration('rs_depth_topic'),
            'kinect_color_topic': LaunchConfiguration('kinect_color_topic'),
            'kinect_depth_topic': LaunchConfiguration('kinect_depth_topic'),
            'kinect2_color_topic': LaunchConfiguration('kinect2_color_topic'),
            'kinect2_depth_topic': LaunchConfiguration('kinect2_depth_topic'),

            'tactile_topic': LaunchConfiguration('tactile_topic'),
            'ft_topic': LaunchConfiguration('ft_topic'),

            'arm_joint_topic': LaunchConfiguration('arm_joint_topic'),
            'require_ee': LaunchConfiguration('require_ee'),
            'base_odom_topic': LaunchConfiguration('base_odom_topic'),

            'img_stamp_mode': LaunchConfiguration('img_stamp_mode'),
            'max_stamp_skew': LaunchConfiguration('max_stamp_skew'),
            'rs_prewarm_frames': LaunchConfiguration('rs_prewarm_frames'),
            'kinect_prewarm_frames': LaunchConfiguration('kinect_prewarm_frames'),
            'tactile_prewarm_frames': LaunchConfiguration('tactile_prewarm_frames'),
        }]
    )

    return LaunchDescription(declared_arguments + [data_record_node])
