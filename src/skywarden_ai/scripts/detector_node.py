#!/usr/bin/env python3
from skywarden_ai.cv_utils import imgmsg_to_cv2, draw_sniper_scope, draw_sniper_scope_better, draw_target_crosshair
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from ament_index_python.packages import get_package_share_directory
from skywarden_interfaces.msg import TargetState
import os
import time

import matplotlib

matplotlib.use("Agg")  # Forces Matplotlib to render headlessly, disabling Qt
from ultralytics import YOLO
from ultralytics.engine.results import Results


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
        self.target_state_pub = self.create_publisher(TargetState, "/target_state", 10)
        self.last_result_time = time.monotonic()
        
        self.id_lock: int = -1

    # todo turn to proper FSM
    def image_callback(self, msg: Image):
        frame = imgmsg_to_cv2(msg)
        result = self.model.track(frame,
            persist=True,
            tracker=self.tracker_config_path,
            verbose=False
        )[0]
        
        fps = 1 / (time.monotonic() - self.last_result_time)
        self.last_result_time = time.monotonic()
        
        target_cx = target_cy = None
        target_state_msg = TargetState()
        
        if result.boxes is None or len(result.boxes) == 0:
            target_state_msg.is_target_visible = False
            self.id_lock = -1
        
        else:
            did_find_track = self.id_lock != -1
            if self.id_lock == -1:
                did_find_track = self.decide_lock(result, frame.shape[0], frame.shape[1])
                
            target_box = None
            if not did_find_track:
                target_box = self.pick_largest_box(result)
            else:
                target_box = result.boxes[result.boxes.id == self.id_lock]
            
            if target_box:
                x1, y1, x2, y2 = map(int, target_box.xyxy.squeeze())
                target_cx = (x1 + x2) // 2
                target_cy = (y1 + y2) // 2

                target_state_msg.is_target_visible = True
                
                frame_height, frame_width = frame.shape[0], frame.shape[1]
                cy, cx = frame_height / 2, frame_width / 2
                
                target_state_msg.norm_dx = (cx - target_cx) / frame_width
                target_state_msg.norm_dy = (cy - target_cy) / frame_height
                target_state_msg.depth_meters = -1.0
                
            else:
                target_state_msg.is_target_visible = False
                self.id_lock = -1
        
        self.target_state_pub.publish(target_state_msg)

        if self.debug:
            annotated_img = result.plot()

            if target_cx is not None and target_cy is not None:
                draw_target_crosshair(annotated_img, target_cx, target_cy)
            
            annotated_img = draw_sniper_scope_better(annotated_img)
            cv2.putText(annotated_img, str(int(fps)), (7, 29), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
            
            cv2.imshow("frame", annotated_img)
            cv2.waitKey(1)
            
    def decide_lock(self, result: Results, img_height: int, img_width: int):
        if result.boxes is None:
            return
            
        cx, cy = img_width / 2, img_height / 2
        min_dist = float('inf')
        did_find_track = False
        
        for box in result.boxes:
            if box.id == None:
                continue
            
            did_find_track = True
            x1, y1, x2, y2 = map(int, box.xyxy.squeeze())
            box_cx = (x1 + x2) / 2
            box_cy = (y1 + y2) / 2
            
            dist = ((box_cx - cx)**2 + (box_cy - cy)**2)**0.5
            if dist < min_dist:
                min_dist = dist
                self.id_lock = int(box.id.item())
                
        return did_find_track
                
                
    def pick_largest_box(self, result: Results):
        if result.boxes is None:
            return
            
        largest_area = float('-inf')
        largest_box = None
        
        for box in result.boxes:
            width, height = box.xywh[0][2], box.xywh[0][3]
            area = width * height
            
            if area > largest_area:
                largest_area = area
                largest_box = box
                
        return largest_box
                
            
def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
