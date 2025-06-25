from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

    default_config_topics = os.path.join(get_package_share_directory('smb_bringup'), 'config', 'twist_mux_topics.yaml')
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_ground_truth',
            default_value='true',
            description='Use ground truth odometry (true/false). If false, launches DLIO.'
        ),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    kinematics_controller = Node(
        package="smb_kinematics",
        executable="smb_kinematics_node",
        name="smb_kinematics_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    low_level_controller = Node(
        package="smb_low_level_controller_gazebo",
        executable="smb_low_level_controller_gazebo_node",
        name="smb_low_level_controller_gazebo_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    smb_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_ui"),
                "launch",
                "smb_ui_sim.launch.py"
            ])
        ),
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/cmd_vel')},
        parameters=[
            {'use_sim_time': use_sim_time},
            LaunchConfiguration('config_topics')]
    )

    return LaunchDescription([
        *launch_args,
        twist_mux,
        kinematics_controller,
        low_level_controller,
        # smb_ui
    ])

