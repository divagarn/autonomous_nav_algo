#!/usr/bin/env python3
"""Software stand-in for the buggy's ECU, so the live stack can be tested in
Gazebo without the real vehicle.

Reads the frames uart_writer *would* send (/uart/frame: yellow, red, angle) and
behaves like the hardware contract documented in this project:
  * red=1            -> stop (after react_sec, then constant decel)
  * red=0, yellow=1  -> 2 km/h preset,   red=0, yellow=0 -> 4 km/h preset
  * after GO the vehicle only starts moving after start_delay_sec (the real
    ECU shows a ~3.5-4 s ramp-up), then accelerates at accel
  * steering follows the command with a slew-rate limit
It publishes the feedback the real ECU would (/wheel_uart: steering deg +right,
speed whole km/h) and drives the Gazebo vehicle through sim_ackermann_bridge
(/sim_ecu/steering_angle, /sim_ecu/vehicle_speed).

THE NUMBERS ARE ASSUMPTIONS, not measurements of the real buggy (override with
parameters). They only exist to exercise the control/stop logic in closed loop.
"""
import rclpy
from ground_segmentation_msgs.msg import SteerSpeed
from rclpy.node import Node
from std_msgs.msg import Float32, String


class EcuSim(Node):
    def __init__(self):
        super().__init__('ecu_sim')
        p = self.declare_parameter
        self.start_delay = p('start_delay_sec', 3.5).value
        self.react = p('react_sec', 0.4).value
        self.accel = p('accel', 0.5).value
        self.decel = p('decel', 1.2).value
        self.slew = p('steer_slew_deg_s', 30.0).value
        self.max_steer = p('max_steering_deg', 20.0).value

        self.red, self.yellow, self.cmd_steer = True, False, 0.0
        self.v, self.steer = 0.0, 0.0
        self.t = 0.0
        self.go_since, self.red_since = None, 0.0
        self.last_frame = -10.0
        self.create_subscription(String, '/uart/frame', self.on_frame, 10)
        self.fb = self.create_publisher(SteerSpeed, '/wheel_uart', 10)
        self.steer_pub = self.create_publisher(Float32, '/sim_ecu/steering_angle', 1)
        self.speed_pub = self.create_publisher(Float32, '/sim_ecu/vehicle_speed', 1)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_feedback)
        self.get_logger().warn('ECU SIMULATOR: assumed start_delay/react/accel/decel (not measured)')

    def on_frame(self, m):
        kv = dict(i.split('=') for i in m.data.split(';'))
        red, yellow = kv['red'] == '1', kv['yellow'] == '1'
        if red and not self.red:
            self.red_since = self.t
        if not red and self.red:
            self.go_since = self.t
        self.red, self.yellow = red, yellow
        self.cmd_steer = max(-self.max_steer, min(self.max_steer, float(kv['angle'])))
        self.last_frame = self.t

    def step(self):
        self.t += self.dt
        target = (2.0 if self.yellow else 4.0) / 3.6
        if self.red:
            if self.t - self.red_since > self.react:
                self.v = max(0.0, self.v - self.decel * self.dt)
        elif self.go_since is not None and self.t - self.go_since > self.start_delay:
            if self.v < target:
                self.v = min(target, self.v + self.accel * self.dt)
            else:
                self.v = max(target, self.v - self.decel * self.dt)
        d = self.slew * self.dt
        self.steer += max(-d, min(d, self.cmd_steer - self.steer))
        self.steer_pub.publish(Float32(data=float(self.steer)))
        self.speed_pub.publish(Float32(data=float(self.v * 3.6)))

    def publish_feedback(self):
        m = SteerSpeed()
        m.steering, m.speed = float(self.steer), float(round(self.v * 3.6))
        self.fb.publish(m)


def main():
    rclpy.init()
    rclpy.spin(EcuSim())


if __name__ == '__main__':
    main()
