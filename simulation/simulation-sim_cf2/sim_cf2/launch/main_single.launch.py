import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Default values
    world_name_default = [FindPackageShare("sim_cf2"), "/worlds/basic.world"] # os.path.join(FindPackageShare("sim_cf2"), "worlds", "basic.world")
    mav_name_default = "crazyflie"
    cf_prefix_default = "cf"

    # Arguments
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value=world_name_default,
        description="Path to the world file",
    )
    mav_name_arg = DeclareLaunchArgument(
        "mav_name", default_value=mav_name_default, description="MAV name"
    )
    cf_prefix_arg = DeclareLaunchArgument(
        "cfPrefix", default_value=cf_prefix_default, description="Prefix for Crazyflie namespaces"
    )

    enable_logging_arg = DeclareLaunchArgument(
        "enable_logging", default_value="false", description="Enable logging"
    )
    enable_parameters_arg = DeclareLaunchArgument(
        "enable_parameters", default_value="false", description="Enable parameters"
    )
    use_ros_time_arg = DeclareLaunchArgument(
        "use_ros_time", default_value="true", description="Use ROS time"
    )
    robot_file_sub_arg = DeclareLaunchArgument(
        "robot_file_sub",
        default_value=[LaunchConfiguration("mav_name"), "_base.xacro"],
        description="Robot file name",
    )

    # Crazyflie 1 specific arguments
    tf_prefix_1_arg = DeclareLaunchArgument(
        "tf_prefix_1",
        default_value=[LaunchConfiguration("cfPrefix"), "1"],
        description="TF prefix for Crazyflie 1",
    )
    color_prop_front_1_arg = DeclareLaunchArgument(
        "color_prop_front_1", default_value="Green", description="Front propeller color"
    )
    color_prop_back_1_arg = DeclareLaunchArgument(
        "color_prop_back_1", default_value="Green", description="Back propeller color"
    )
    x_1_arg = DeclareLaunchArgument(
        "x_1", default_value="0.0", description="Initial x position for Crazyflie 1"
    )
    y_1_arg = DeclareLaunchArgument(
        "y_1", default_value="0.0", description="Initial y position for Crazyflie 1"
    )
    z_1_arg = DeclareLaunchArgument(
        "z_1", default_value="0.03", description="Initial z position for Crazyflie 1"
    )
    enable_mr_deck_1_arg = DeclareLaunchArgument(
        "enable_mr_deck_1",
        default_value="true",
        description="Enable MR deck for Crazyflie 1",
    )
    mr_deck_visualize_1_arg = DeclareLaunchArgument(
        "mr_deck_visualize_1",
        default_value="true",
        description="Visualize MR deck for Crazyflie 1",
    )

    # Include the Gazebo world launch
    start_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("sim_cf2"), "/launch/start_gazebo_world.launch.py"]
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name")
        }.items(),
    )

    # Include Crazyflie 1 launch
    spawn_crazyflie_1 = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [FindPackageShare("sim_cf2"), "/launch/spawn_mav.launch.xml"]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("tf_prefix_1"),
            "mav_name": LaunchConfiguration("mav_name"),
            "robot_file": LaunchConfiguration("robot_file_sub"),
            "tf_prefix": LaunchConfiguration("tf_prefix_1"),
            "color_prop_front": LaunchConfiguration("color_prop_front_1"),
            "color_prop_back": LaunchConfiguration("color_prop_back_1"),
            "x_initial": LaunchConfiguration("x_1"),
            "y_initial": LaunchConfiguration("y_1"),
            "z_initial": LaunchConfiguration("z_1"),
            "roll_initial": "0.0",
            "pitch_initial": "0.0",
            "yaw_initial": "0.0",
            "enable_mr_deck": LaunchConfiguration("enable_mr_deck_1"),
            "mr_deck_visualize": LaunchConfiguration("mr_deck_visualize_1"),
        }.items(),
    )

    # Return launch description
    return LaunchDescription(
        [
            world_name_arg,
            mav_name_arg,
            cf_prefix_arg,
            enable_logging_arg,
            enable_parameters_arg,
            use_ros_time_arg,
            robot_file_sub_arg,
            tf_prefix_1_arg,
            color_prop_front_1_arg,
            color_prop_back_1_arg,
            x_1_arg,
            y_1_arg,
            z_1_arg,
            enable_mr_deck_1_arg,
            mr_deck_visualize_1_arg,
            start_gazebo_world,
            spawn_crazyflie_1,
        ]
    )
