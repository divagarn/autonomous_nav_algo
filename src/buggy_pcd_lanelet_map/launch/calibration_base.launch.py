"""Minimum stack for the brake calibration tool (no map, RViz or NDT):
ECU feedback (/wheel_uart), wheel odometry (/odom) and the only serial writer.

  ./buggy calibrate-brake [--preset 4] [--live]      (starts and stops this for you)

loopback:=true  (default) uart_writer logs frames, writes nothing
loopback:=false uart_writer REALLY WRITES to sender_port
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('feedback_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('sender_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('loopback', default_value='true'),
        Node(package='buggy_uart', executable='uart_bridge', name='uart_bridge',
             output='screen',
             parameters=[{'port': LaunchConfiguration('feedback_port'), 'baud': 115200}]),
        Node(package='buggy_perception', executable='wheel_odometry', name='wheel_odometry',
             output='screen', parameters=[{'publish_tf': False}]),
        Node(package='buggy_pcd_lanelet_map', executable='uart_writer', name='uart_writer',
             output='screen',
             parameters=[{'port': LaunchConfiguration('sender_port'), 'baudrate': 115200,
                          'loopback': ParameterValue(LaunchConfiguration('loopback'),
                                                     value_type=bool)}]),
    ])
