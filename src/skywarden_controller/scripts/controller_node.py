#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum, auto
from skywarden_interfaces.msg import TargetState
from std_msgs.msg import Float64MultiArray
import time


class Controller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral_cap = 2.5
        self.integral_decay = 0.98
        
        self.last_compute_time = time.monotonic()
        self.integral_sum = 0.0
        self.last_err = 0

    def compute(self, err):
        now = time.monotonic()
        dt = now - self.last_compute_time
        self.last_compute_time = now
        
        p_out = self.kp * err

        self.integral_sum = max(min((self.integral_sum * self.integral_decay) + err * dt, self.integral_cap), -self.integral_cap)
        i_out = self.ki * self.integral_sum
        
        d_err = (err - self.last_err) / dt
        d_out = self.kd * d_err
        self.last_err = err

        print(f"p_term: {p_out:.2f} i_term: {i_out:.2f}, integral_sum: {self.integral_sum:.2f} d_out: {d_out:.6f}")

        return p_out + i_out + d_out

    def clear(self):
        self.integral_sum = 0
        self.last_err = 0
        self.last_compute_time = time.monotonic()


class SystemState(Enum):
    IDLE = auto()
    LOCKING = auto()
    SHOOT = auto()


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.get_logger().info("controller_node started!")

        self.yaw_controller =   Controller(0.7, 0.5, 0.05)
        self.pitch_controller = Controller(0.7, 0.5, 0.05)

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

        self.yaw_controller.clear()
        self.pitch_controller.clear()
        self.get_logger().info("Switching to locking state")
        self.state = SystemState.LOCKING

    def locking(self):
        if (time.monotonic() - self.last_msg_time) > 2.5 or (self.target_state_msg is None):
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
        cmd_yaw = self.yaw_controller.compute(-err_x)
        cmd_pitch = self.pitch_controller.compute(err_y)

        # Publish commands
        msg = Float64MultiArray()
        msg.data = [float(cmd_yaw), float(cmd_pitch)]
        self.joint_vel_publisher.publish(msg)

    def target_state_callback(self, msg):
        if not msg.is_target_visible:
            return

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
