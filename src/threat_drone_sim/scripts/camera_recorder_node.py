#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time
import os

def imgmsg_to_cv2(msg):
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == "rgb8": # it's this
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")


class CameraRecorderNode(Node):
    def __init__(self):
        super().__init__("camera_recorder_node")
        self.get_logger().info(f"camera_recorder_node Started")

        self.save_path = "/home/zeyadcode/skywarden_ws/src/threat_drone_sim/gz_footage"
        self.image_sub_ = self.create_subscription(Image, "/camera/image", self.image_callback, 10)
        self.last_save = time.monotonic()
        self.frame_idx = 0
        
    def image_callback(self, msg: Image):
        frame = imgmsg_to_cv2(msg)
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        
        # if time.monotonic() - self.last_save >= 1:
        #     cv2.imwrite(os.path.join(self.save_path, f"img_{self.frame_idx}.png"), frame)
        #     self.frame_idx += 1
            
        
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()