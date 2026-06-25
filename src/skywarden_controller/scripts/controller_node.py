#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum, auto

# Make sure to source your workspace so this custom message is found
from skywarden_interfaces.msg import TargetState
from std_msgs.msg import Float64MultiArray
import time

class SystemState(Enum):
    IDLE = auto()
    LOCKING = auto()
    SHOOT = auto()


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.get_logger().info("controller_node started!")

        self.declare_parameter("Kyaw", 2.0)
        self.declare_parameter("Kpitch", 2.0)

        self.Kyaw = self.get_parameter("Kyaw").value
        self.Kpitch = self.get_parameter("Kpitch").value

        self.target_state_sub = self.create_subscription(TargetState, "/target_state", self.target_state_callback, 10)
        self.joint_vel_publisher = self.create_publisher(Float64MultiArray, "/joint_trajectory_controller/commands", 10)

        # Timer (100ms = 0.1 seconds)
        self.main_timer = self.create_timer(0.1, self.main_loop)

        self.state = SystemState.IDLE
        self.target_state_msg = None

        self.last_msg_time = time.monotonic()

    def main_loop(self):
        if self.state == SystemState.IDLE:
            self.idle()
        elif self.state == SystemState.LOCKING:
            self.locking()

    def idle(self):
        if self.target_state_msg is None or not self.target_state_msg.is_target_visible:
            return

        self.get_logger().info("Switching to locking state")
        self.state = SystemState.LOCKING

    def locking(self):
        time_diff = time.monotonic() - self.last_msg_time # seconds

        if time_diff > 0.2 or self.target_state_msg is None:
            self.get_logger().warn("Target signal lost! Returning to IDLE.")
            self.state = SystemState.IDLE
            self.target_state_msg = None

            # Publish zero velocities
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self.joint_vel_publisher.publish(msg)
            return

        # Extract errors
        err_x = self.target_state_msg.norm_err_x
        err_y = self.target_state_msg.norm_err_y

        # Calculate control commands
        cmd_yaw = self.Kyaw * -err_x
        cmd_pitch = self.Kpitch * err_y

        # Publish commands
        msg = Float64MultiArray()
        msg.data = [float(cmd_yaw), float(cmd_pitch)]
        self.joint_vel_publisher.publish(msg)

    def target_state_callback(self, msg):
        self.last_msg_time = time.monotonic()
        self.target_state_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
