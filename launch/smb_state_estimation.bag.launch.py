# NOTE: TO BE USED WITH A RECORDED ROS BAG/Real robot?

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
            default_value='false',
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    low_level_controller = Node(
        package="smb_low_level_controller_gazebo",
        executable="smb_low_level_controller_gazebo_node",
        name="smb_low_level_controller_gazebo_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
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

    smb_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_ui"),
                "launch",
                "smb_ui_sim.launch.py"
            ])
        ),
    )

    ############### STATE ESTIMATION ####################

    # This works in the odom_graph_msf frame
    graph_msf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("smb_estimator_graph_ros2"),
                "launch",
                "smb_estimator_graph_replay.launch.py"
            ])
        ),
    )

    open_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("open3d_slam_ros"),
                "launch",
                "summer_school_slam_robot_launch.py"
            ])
        ),
    )

    # Terrain analysis node
    terrain_analysis = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrainAnalysis",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Terrain analysis ext node
    terrain_analysis_ext = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrainAnalysisExt",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        *launch_args,
        twist_mux,
        kinematics_controller,
        low_level_controller,
        graph_msf_launch,
        open_slam_launch,
        terrain_analysis,
        terrain_analysis_ext,
    ])


