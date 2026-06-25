#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class DroneMoverNode(Node):
    def __init__(self):
        super().__init__("orbit_drone")

        self.declare_parameter("orbit_radius", 4.18)
        self.declare_parameter("drone_speed", 1.0)  # your requested default

        self.radius = float(self.get_parameter("orbit_radius").value)  # type: ignore
        self.speed = float(self.get_parameter("drone_speed").value)  # type: ignore

        self.pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.timer = self.create_timer(1.0 / 50, self.step)

        self.get_logger().info(f"Orbiting radius={self.radius:.3f} m | speed={self.speed:.3f} m/s")

    def step(self):
        if self.radius <= 1e-6:
            return

        msg = Twist()

        # Forward speed in the drone's body frame
        msg.linear.x = self.speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Constant angular speed for circular motion: omega = v / r
        omega = self.speed / self.radius
        msg.angular.z = omega

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneMoverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
