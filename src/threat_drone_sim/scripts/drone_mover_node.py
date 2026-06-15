#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class DroneMoverNode(Node):
    def __init__(self):
        super().__init__("drone_mover")
        self.get_logger().info(f"node_name Started")

        self.forward_speed = 0.5
        self.hz = 30
        
        self.publisher_ = self.create_publisher(Twist, "/parrot_bebop/cmd_vel", 10)
        self.timer_ = self.create_timer(1 / self.hz, self.update_position)
        self.time_elapsed = 0

    def update_position(self):
        self.time_elapsed += 1 / self.hz

        msg = Twist()
        msg.linear.x = self.forward_speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneMoverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
