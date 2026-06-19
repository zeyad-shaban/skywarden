#!/usr/bin/env python3
from cv_utils import imgmsg_to_cv2
import rclpy
from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from sensor_msgs.msg import Image
import cv2
from ament_index_python.packages import get_package_share_directory
import os

import matplotlib

matplotlib.use("Agg")  # Forces Matplotlib to render headlessly, disabling Qt
from ultralytics import YOLO


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.get_logger().info(f"detector_node Started")

        self.declare_parameter("debug", True)
        self.debug: bool = self.get_parameter("debug")  # type: ignore

        model_path = os.path.join(get_package_share_directory("skywarden_ai"), "static", "models", "yolo26n_640.pt")
        self.model = YOLO(model_path)

        self.img_sub_ = self.create_subscription(Image, "/camera/image", self.image_callback, 10)

    def image_callback(self, msg: Image):
        frame = imgmsg_to_cv2(msg)

        results = self.model(frame)

        if self.debug:
            annotated_img = results[0].plot()
            cv2.imshow("frame", annotated_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
