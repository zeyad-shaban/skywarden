#!/usr/bin/env python3
from skywarden_ai.cv_utils import imgmsg_to_cv2, draw_sniper_scope, draw_sniper_scope_better
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from ament_index_python.packages import get_package_share_directory
import os
import time

import matplotlib

matplotlib.use("Agg")  # Forces Matplotlib to render headlessly, disabling Qt
from ultralytics import YOLO


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.get_logger().info(f"detector_node Started")

        self.declare_parameter("debug", True)
        self.debug: bool = self.get_parameter("debug").value  # type: ignore

        static_path = os.path.join(get_package_share_directory("skywarden_ai"), "static")
        self.tracker_config_path = os.path.join(static_path, "config", "botsort_config.yaml")
        
        model_path = os.path.join(static_path, "models", "yolo26n_640_int8_openvino_model")
        self.model = YOLO(model_path)

        self.img_sub_ = self.create_subscription(Image, "/camera/image", self.image_callback, 10)
        self.last_result_time = time.monotonic()

    def image_callback(self, msg: Image):
        frame = imgmsg_to_cv2(msg)
        results = self.model.track(frame,
            persist=True,
            tracker=self.tracker_config_path,
            verbose=False,)
        
        fps = 1 / (time.monotonic() - self.last_result_time)
        self.last_result_time = time.monotonic()

        if self.debug:
            annotated_img = results[0].plot()
            annotated_img = draw_sniper_scope(annotated_img)

            cv2.putText(annotated_img, str(int(fps)), (7, 29), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
            
            cv2.imshow("frame", annotated_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
