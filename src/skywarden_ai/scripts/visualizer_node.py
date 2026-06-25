#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from skywarden_ai.cv_utils import CompressedImage
from skywarden_ai.cv_utils import compressed_imgmsg_to_cv2, draw_sniper_scope_better, draw_target_crosshair
from skywarden_interfaces.msg import TargetState
import cv2
import time

class VisualizerNode(Node):
    def __init__(self):
        super().__init__("visualizer_node")
        self.get_logger().info(f"visualizer_node Started")
        
        self.img_sub_ = self.create_subscription(CompressedImage, "/camera/annotated/compressed", self.image_callback, 10)
        self.fps_sub_ = self.create_subscription(TargetState, "/target_state", self.target_state_callback, 10)
        self.norm_err_x = self.target_cy = None
        
        self.avg_fps = 3
        self.last_time: float = time.perf_counter()
        
        self.ema_alpha = 0.1
        
    def image_callback(self, msg: CompressedImage):
        annotated_img = compressed_imgmsg_to_cv2(msg)
        if annotated_img is None:
            self.get_logger().warning("Couldn't extract image from msg")
            return

        if self.norm_err_x is not None and self.target_cy is not None:
            h, w = annotated_img.shape[0], annotated_img.shape[1]
            target_cx = int((w / 2) + self.norm_err_x * w)
            target_cy = int((h / 2) + self.norm_err_y * h)
            draw_target_crosshair(annotated_img, target_cx, target_cy)
        
        annotated_img = draw_sniper_scope_better(annotated_img)
        cv2.putText(annotated_img, str(int(self.avg_fps)), (7, 29), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
        
        cv2.imshow("frame", annotated_img)
        cv2.waitKey(1)
        
    def target_state_callback(self, msg: TargetState):
        fps = 1/(time.perf_counter() - self.last_time)
        self.last_time = time.perf_counter()
        self.avg_fps = (1 - self.ema_alpha) * self.avg_fps + self.ema_alpha * fps
        
        self.norm_err_x = msg.norm_err_x
        self.norm_err_y = msg.norm_err_y


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()