#!/usr/bin/env python3
"""UART writer for the buggy ECU, with a watchdog (the ONLY node that writes).

Subscribes the same topics uart_sender_node does - /steering_angle [deg,
+right], /final_red_detected (stop), /final_yellow_detected (speed preset:
yellow=1 -> 2 km/h, yellow=0 -> 4 km/h) - and sends the 6-byte frame
`( yellow red angle )` via buggy_uart's UartInterface at 10 Hz.

Differences from uart_sender_node, deliberately:
  * WATCHDOG: if any of the three topics is older than watchdog_sec the frame
    is forced to STOP (red=1, steering 0). uart_sender_node has that logic
    commented out and keeps sending its last command if its publisher dies.
  * Starts in STOP and sends STOP on shutdown.
  * loopback defaults to TRUE: builds and logs every frame but opens no port
    and writes nothing. Real writing needs loopback:=false explicitly.
Never run together with uart_sender_node (two writers on one port).

Publishes /uart/frame ("yellow=0;red=1;angle=0.00", every frame sent) and /uart/status ("mode=loopback|real;state=ok|stale;frames=N") at 2 Hz.
"""
import rclpy
from buggy_uart.uart_interface import UartInterface
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class UartWriter(Node):
    def __init__(self):
        super().__init__('uart_writer')
        port = self.declare_parameter('port', '/dev/ttyUSB0').value
        baud = self.declare_parameter('baudrate', 115200).value
        self.loopback = self.declare_parameter('loopback', True).value
        self.watchdog = self.declare_parameter('watchdog_sec', 0.5).value

        self.uart = UartInterface(port, baud, loopback=self.loopback)
        self.steer, self.red, self.yellow = 0.0, True, False
        now = self.get_clock().now()
        self.t_steer = self.t_red = self.t_yellow = now
        self.frames = 0
        self.stale = True
        self.create_subscription(Float32, '/steering_angle', self.on_steer, 1)
        self.create_subscription(Bool, '/final_red_detected', self.on_red, 1)
        self.create_subscription(Bool, '/final_yellow_detected', self.on_yellow, 1)
        self.status_pub = self.create_publisher(String, '/uart/status', 1)
        self.frame_pub = self.create_publisher(String, '/uart/frame', 10)
        self.create_timer(0.1, self.send)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().warn(
            f'uart_writer on {port}: ' + ('LOOPBACK (nothing is written)' if self.loopback
                                          else 'REAL WRITE ENABLED'))

    def on_steer(self, m):
        self.steer, self.t_steer = float(m.data), self.get_clock().now()

    def on_red(self, m):
        self.red, self.t_red = bool(m.data), self.get_clock().now()

    def on_yellow(self, m):
        self.yellow, self.t_yellow = bool(m.data), self.get_clock().now()

    def age(self, t):
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def send(self):
        stale = max(self.age(self.t_steer), self.age(self.t_red),
                    self.age(self.t_yellow)) > self.watchdog
        if stale != self.stale:
            self.get_logger().warn('commands STALE - forcing STOP' if stale
                                   else 'commands fresh again')
        self.stale = stale
        yellow, red, steer = (False, True, 0.0) if stale else (self.yellow, self.red, self.steer)
        self.uart.send(yellow, red, steer)
        # what was (or, in loopback, would have been) sent - ecu_sim and logging use this
        self.frame_pub.publish(String(data=f'yellow={int(yellow)};red={int(red)};angle={steer:.2f}'))
        self.frames += 1

    def publish_status(self):
        self.status_pub.publish(String(data=(
            f"mode={'loopback' if self.loopback else 'real'};"
            f"state={'stale' if self.stale else 'ok'};frames={self.frames}")))

    def shutdown(self):
        try:
            self.get_logger().warn('shutdown: sending final STOP')
            self.uart.send(False, True, 0.0)
        finally:
            self.uart.close()


def main():
    rclpy.init()
    node = UartWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
