"""Real-hardware chcnav GPS/INS bringup: raw HC CGI protocol arrives over
UDP from the CGI device (see chcnav/doc/README-en.md section 2.3 for the
device-side UDP config) -> HcMsgParserLaunchNode splits it into NMEA/HC
frames on /chcnav/nmea_sentence + /chcnav/hc_sentence -> HcCgiProtocolProcessNode
CRC32-validates and decodes HCINSPVATZCB/HCRAWIMUIB into /chcnav/devpvt +
/chcnav/devimu -> ChcnavFixDemo converts /chcnav/devpvt into standard
sensor_msgs/NavSatFix (/fix) and sensor_msgs/Imu (/imu).

Sim counterpart: gps_sim.launch.py, in this same package. That one runs no
chcnav code at all - it spawns Gazebo with the GPS/IMU sensor plugins wired
onto gps_mount_link/imu_mount_link in buggy_description's URDF, publishing
the same /fix and /imu topics, so anything downstream is agnostic to which
of these two launch files brought the topics up.

SAFETY: verify udp_port matches the port actually configured in the CGI
device's I/O Settings (TCP/UDP_Client 7) and that the device is sending to
this PC's IP - see the UDP_IP_地址 screenshot in chcnav/doc/README-en.md
section 2.3. This launch only reads; it has no write-back path (that's what
ntrip_server_node's remap to the parser's "write" topic is for, see demo_6).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='7531',
        description='UDP port the CGI device is configured to send to (chcnav/doc/README-en.md 2.3).')
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='1000',
        description='Max protocols parsed per second (HcMsgParserLaunchNode "rate" param).')

    hc_topic_driver = Node(
        package='chcnav',
        executable='HcCgiProtocolProcessNode',
        name='hc_topic_driver',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    udp_parser = Node(
        package='chcnav',
        executable='HcMsgParserLaunchNode',
        name='udp_gps',
        output='screen',
        parameters=[{
            'type': 'udp',
            'rate': LaunchConfiguration('rate'),
            'port': LaunchConfiguration('udp_port'),
            'use_sim_time': False,
        }],
    )

    chcnav_fix_demo = Node(
        package='chcnav',
        executable='ChcnavFixDemo',
        name='chcnav_fix_demo',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        udp_port_arg,
        rate_arg,
        hc_topic_driver,
        udp_parser,
        chcnav_fix_demo,
    ])
