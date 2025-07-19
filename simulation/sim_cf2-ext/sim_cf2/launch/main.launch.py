import os
import yaml
import re

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Helper method to load YAML file
def load_robots_config(config_path):
    """
    Load robots configuration from a YAML file if it exists, else return an empty dictionary.
    """
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            try:
                return yaml.safe_load(file)
            except yaml.YAMLError as e:
                print(f"Error loading YAML file: {e}")
                return {}
    else:
        print(f"YAML config file not found at {config_path}. Using default settings.")
        return {}

def generate_launch_description():

    # Load the drone configuration from YAML
    config_path = os.path.join(get_package_share_directory('sim_cf2'), 'launch', 'crazyflies.yaml')
    robots_config = load_robots_config(config_path)

    # Default values
    world_name_default = [FindPackageShare("sim_cf2"), "/worlds/basic.world"]
    mav_name_default = "crazyflie"
    cf_prefix_default = "cf"

    # Declare launch arguments
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

    # Include the Gazebo world launch
    start_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("sim_cf2"), "/launch/start_gazebo_world.launch.py"]
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name")
        }.items(),
    )

    # Initialize lists for spawn actions
    drone_launches = []

    # Iterate over each robot in the YAML configuration
    for robot_name, robot_config in robots_config.get('robots', {}).items():
        if robot_config.get('enabled', False):
            spawn_actions = []
            robot_name_prefix = int(re.search(r'\d+', robot_name).group()) if re.search(r'\d+', robot_name) else None
            
            tf_prefix = robot_config.get('tf_prefix', robot_name_prefix)
            color_prop_front = robot_config.get('color_prop_front', "Green")
            color_prop_back = robot_config.get('color_prop_back', "Green")
            x_initial = robot_config.get('initial_position', [0.0, 0.0, 0.03])[0]
            y_initial = robot_config.get('initial_position', [0.0, 0.0, 0.03])[1]
            z_initial = robot_config.get('initial_position', [0.0, 0.0, 0.03])[2]
            enable_mr_deck = robot_config.get('enable_mr_deck', "true")
            mr_deck_visualize = robot_config.get('mr_deck_visualize', "true")
            roll_initial = robot_config.get('initial_orientation', {}).get('roll', 0.0)
            pitch_initial = robot_config.get('initial_orientation', {}).get('pitch', 0.0)
            yaw_initial = robot_config.get('initial_orientation', {}).get('yaw', 0.0)
            
            drone_args = [
                DeclareLaunchArgument(
                    f"tf_prefix_{robot_name}",
                    default_value=[LaunchConfiguration("cfPrefix"), str(robot_name_prefix)],
                    description=f"TF prefix for Crazyflie {robot_name} {robot_name_prefix}",
                ),
                DeclareLaunchArgument(
                    f"color_prop_front_{robot_name}", 
                    default_value=color_prop_front, 
                    description=f"Front propeller color for Crazyflie {robot_name}"
                ),
                DeclareLaunchArgument(
                    f"color_prop_back_{robot_name}", 
                    default_value=color_prop_back, 
                    description=f"Back propeller color for Crazyflie {robot_name}"
                ),
                DeclareLaunchArgument(
                    f"x_{robot_name}", 
                    default_value=str(x_initial), 
                    description=f"Initial x position for Crazyflie {robot_name}"
                ),
                DeclareLaunchArgument(
                    f"y_{robot_name}", 
                    default_value=str(y_initial), 
                    description=f"Initial y position for Crazyflie {robot_name}"
                ),
                DeclareLaunchArgument(
                    f"z_{robot_name}", 
                    default_value=str(z_initial), 
                    description=f"Initial z position for Crazyflie {robot_name}"
                ),
                DeclareLaunchArgument(
                    f"enable_mr_deck_{robot_name}", 
                    default_value=str(enable_mr_deck), 
                    description=f"Enable MR deck for Crazyflie {robot_name}"
                ),
                DeclareLaunchArgument(
                    f"mr_deck_visualize_{robot_name}", 
                    default_value=str(mr_deck_visualize), 
                    description=f"Visualize MR deck for Crazyflie {robot_name}"
                ),
            ]

            spawn_crazyflie = IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [FindPackageShare("sim_cf2"), "/launch/spawn_mav.launch.xml"]
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration(f"tf_prefix_{robot_name}", default=[LaunchConfiguration("cfPrefix"), str(robot_name)]),
                    "mav_name": LaunchConfiguration("mav_name"),
                    "robot_file": LaunchConfiguration("robot_file_sub"),
                    "tf_prefix": LaunchConfiguration(f"tf_prefix_{robot_name}", default=[LaunchConfiguration("cfPrefix"), str(robot_name_prefix)]),
                    #
                    "color_prop_front": LaunchConfiguration(f"color_prop_front_{robot_name}", default=color_prop_front),
                    "color_prop_back": LaunchConfiguration(f"color_prop_back_{robot_name}", default=color_prop_back),
                    "x_initial": LaunchConfiguration(f"x_{robot_name}", default=str(x_initial)),
                    "y_initial": LaunchConfiguration(f"y_{robot_name}", default=str(y_initial)),
                    "z_initial": LaunchConfiguration(f"z_{robot_name}", default=str(z_initial)),
                    "roll_initial": LaunchConfiguration(f"roll_{robot_name}", default=str(roll_initial)),
                    "pitch_initial": LaunchConfiguration(f"pitch_{robot_name}", default=str(pitch_initial)),
                    "yaw_initial": LaunchConfiguration(f"yaw_{robot_name}", default=str(yaw_initial)),
                    "enable_mr_deck": LaunchConfiguration(f"enable_mr_deck_{robot_name}", default=str(enable_mr_deck)),
                    "mr_deck_visualize": LaunchConfiguration(f"mr_deck_visualize_{robot_name}", default=str(mr_deck_visualize)),
                }.items(),
            )

            print(robot_name)
            print(robot_config)
            print("---")

            drone_launches.extend(drone_args)
            drone_launches.append(spawn_crazyflie)
    
    # Return launch description
    return LaunchDescription(
        [
            #
            world_name_arg,
            mav_name_arg,
            cf_prefix_arg,
            enable_logging_arg,
            enable_parameters_arg,
            use_ros_time_arg,
            robot_file_sub_arg,
            #
            start_gazebo_world,
            #
        ] + drone_launches
    )
