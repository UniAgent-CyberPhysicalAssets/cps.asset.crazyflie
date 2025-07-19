import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    crazyflies_yaml = os.path.join(get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml')
    with open(crazyflies_yaml, 'r') as file:
        crazyflies = yaml.safe_load(file)

    # Define the backend argument with a default value
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='cflib',
        description='Backend type for crazyflie server'
    )

    # Load the server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')
    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)
    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]

    # Load the robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    server_params[1]['robot_description'] = robot_desc

    # Define the crazyflie server node with an IfCondition using PythonExpression
    server_node_real = Node(
        package='crazyflie',
        executable='crazyflie_server.py',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('backend'), "' == 'cflib'"])
        ),
        name='crazyflie_server',
        output='screen',
        parameters=server_params,
    )
    server_sim_node = Node(
        package='crazyflie_sim',
        executable='crazyflie_server',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"])
        ),
        name='crazyflie_server',
        output='screen',
        emulate_tty=True,
        parameters= server_params,
    )

    return LaunchDescription([
        backend_arg,
        server_node_real,
        server_sim_node
    ])
