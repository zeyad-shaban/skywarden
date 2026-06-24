#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros
import numpy as np
import math


class TrajectoryPredictorNode(Node):
    def __init__(self):
        super().__init__("trajectory_predictor")
        self.get_logger().info("Trajectory Predictor Started")

        # --- Parameters ---
        self.declare_parameter("projectile_velocity", 15.0)  # Initial speed of bullet (m/s)
        self.declare_parameter("gravity", 9.81)             # Gravity acceleration (m/s^2)
        self.declare_parameter("time_step", 0.05)           # Step size for drawing points (seconds)
        self.declare_parameter("max_time", 2.0)             # How far ahead to predict (seconds)
        self.declare_parameter("world_frame", "footprint")       # Static base frame
        self.declare_parameter("muzzle_frame", "muzzle_link") # Frame at your gun muzzle

        # Get values
        self.v0 = self.get_parameter("projectile_velocity").value
        self.g = self.get_parameter("gravity").value
        self.dt = self.get_parameter("time_step").value
        self.max_t = self.get_parameter("max_time").value
        self.world_frame = self.get_parameter("world_frame").value
        self.muzzle_frame = self.get_parameter("muzzle_frame").value

        # --- TF Buffer & Listener ---
        # Used to dynamically find where the turret is pointing in the world
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        # Publishes the line strip marker to RViz
        self.marker_pub = self.create_publisher(Marker, "/targeting_trajectory", 10)

        # Timer to update the line at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.update_trajectory)

    def update_trajectory(self):
        try:
            # 1. Look up the transformation from the static world to your muzzle joint
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.muzzle_frame,
                now
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Gracefully wait if TF isn't ready yet
            return

        # 2. Extract muzzle position (x0, y0, z0) in world coordinates
        x0 = trans.transform.translation.x
        y0 = trans.transform.translation.y
        z0 = trans.transform.translation.z

        # 3. Extract orientation quaternion
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.z # Wait, quaternions require correct mapping:
        qw_val = trans.transform.rotation.w

        # Compute forward unit vector (X-axis of the muzzle frame) in world coordinates
        # Using Quaternion-to-Vector Rotation formula: V_world = R * [1, 0, 0]^T
        r_x = 1.0 - 2.0 * (qy**2 + qz**2)
        r_y = 2.0 * (qx * qy + qz * qw_val)
        r_z = 2.0 * (qx * qz - qy * qw_val)

        # Normalize the forward vector
        norm = math.sqrt(r_x**2 + r_y**2 + r_z**2)
        forward_vector = np.array([r_x/norm, r_y/norm, r_z/norm])

        # 4. Compute launch velocity vector
        v_vector = forward_vector * self.v0

        # 5. Generate Points along the parabolic/linear trajectory
        marker_points = []
        t = 0.0
        while t <= self.max_t:
            # Ballistic trajectory kinematics equations
            x = x0 + v_vector[0] * t
            y = y0 + v_vector[1] * t
            # Note: Gravity acts downwards along the World Z-axis (-Z)
            z = z0 + v_vector[2] * t - 0.5 * self.g * (t**2)

            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker_points.append(p)

            t += self.dt

        # 6. Publish the Marker
        self.publish_marker(marker_points)

    def publish_marker(self, points):
        marker = Marker()
        # Header parameters
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ballistics"
        marker.id = 0
        
        # We use LINE_STRIP to draw a connected line of points
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Assign points
        marker.points = points

        # Thickness of the line
        marker.scale.x = 0.03  # Line width in meters

        # Color of the line (Semi-transparent neon green, like a laser guide!)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.7  # 70% opacity

        # Ensure marker persists
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()