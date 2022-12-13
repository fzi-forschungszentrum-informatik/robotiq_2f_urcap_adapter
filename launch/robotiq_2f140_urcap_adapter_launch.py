from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.0.104"
    )
    robot_port_launch_arg = DeclareLaunchArgument(
        "robot_port", default_value="63352"
    )

    robotiq_2f_urcap_adapter_node = Node(
            package='robotiq_2f_urcap_adapter',
            namespace='robotiq_2f140_urcap_adapter',
            executable='robotiq_2f_adapter_node.py',
            name='robotiq_2f85_urcap_adapter',
            parameters=[{
                "robot_ip": LaunchConfiguration('robot_ip'),
                "robot_port": LaunchConfiguration('robot_port'),
                "max_gripper_width": 0.14,
                "min_gripper_width": 0.0,
                "max_gripper_speed": 0.25,
                "min_gripper_speed": 0.03,
                "max_gripper_force": 125.0,
                "min_gripper_force": 10.0,
            }]
        )
    return LaunchDescription([
        robot_ip_launch_arg,
        robot_port_launch_arg,
        robotiq_2f_urcap_adapter_node
    ])
