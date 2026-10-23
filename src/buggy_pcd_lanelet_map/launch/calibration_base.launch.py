"""Minimum stack for the brake calibration tool (no map, RViz or NDT):
ECU feedback (/wheel_uart), wheel odometry (/odom) and the only serial writer.

  ./buggy calibrate-brake [--preset 4] [--live]      (starts and stops this for you)

loopback:=true  (default) uart_writer logs frames, writes nothing
loopback:=false uart_writer REALLY WRITES to sender_port
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from buggy_pcd_lanelet_map.paths import config_files


def launch_setup(context):
    # ports: the YAML (config/buggy_params.yaml) unless given on the command line
    cfg = config_files(LaunchConfiguration('params').perform(context))
    fb, sd = LaunchConfiguration('feedback_port').perform(context), LaunchConfiguration('sender_port').perform(context)
    core_cpp = LaunchConfiguration('core').perform(context) == 'cpp'
    pkg_rt = 'buggy_core' if core_cpp else 'buggy_pcd_lanelet_map'
    return [
        Node(package='buggy_core' if core_cpp else 'buggy_uart', executable='uart_bridge', name='uart_bridge',
             output='screen', parameters=[*cfg, {'port': fb} if fb else {}]),
        Node(package='buggy_core' if core_cpp else 'buggy_perception', executable='wheel_odometry',
             name='wheel_odometry', output='screen', parameters=[*cfg, {'publish_tf': False}]),
        Node(package=pkg_rt, executable='uart_writer', name='uart_writer', output='screen',
             parameters=[*cfg, {'loopback': ParameterValue(LaunchConfiguration('loopback'), value_type=bool)},
                         {'port': sd} if sd else {}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('feedback_port', default_value='',
                              description='empty = uart_bridge.port in config/buggy_params.yaml'),
        DeclareLaunchArgument('sender_port', default_value='',
                              description='empty = uart_writer.port in config/buggy_params.yaml'),
        DeclareLaunchArgument('loopback', default_value='true'),
        DeclareLaunchArgument('core', default_value='python', description='python | cpp'),
        DeclareLaunchArgument('params', default_value='', description='your own parameter YAML'),
        OpaqueFunction(function=launch_setup),
    ])
