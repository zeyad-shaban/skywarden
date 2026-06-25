#!/usr/bin/env python3
from skywarden_ai.cv_utils import compressed_imgmsg_to_cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from skywarden_interfaces.msg import TargetState
import os
import time
import cv2

import matplotlib

matplotlib.use("Agg")
from ultralytics import YOLO
from ultralytics.engine.results import Results

class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.get_logger().info(f"detector_node Started")

        self.declare_parameter("drone_width", 0.33) # default parrot_bebop model
        self.declare_parameter("offset_cam_muzzle", 0.06) # default parrot_bebop model
        self.declare_parameter("velocity_projectile", 900.0) # m/s
        self.declare_parameter("Fx", 554.3827128) # default cacluated in gz simulation
        self.declare_parameter("Fy", 554.3827128) # default cacluated in gz simulation
        self.drone_width: float = self.get_parameter("drone_width").value # type: ignore
        self.offset_cam_muzzle: float = self.get_parameter("offset_cam_muzzle").value # type: ignore
        self.velocity_projectile: float = self.get_parameter("velocity_projectile").value # type: ignore
        self.Fx: float = self.get_parameter("Fx").value # type: ignore
        self.Fy: float = self.get_parameter("Fy").value # type: ignore
        
        static_path = os.path.join(get_package_share_directory("skywarden_ai"), "static")
        self.tracker_config_path = os.path.join(static_path, "config", "botsort_config.yaml")
        
        model_path = os.path.join(static_path, "models", "yolo26n_640_int8_openvino_model")
        self.model = YOLO(model_path)

        self.img_sub_ = self.create_subscription(CompressedImage, "/camera/image/compressed", self.image_callback, 10)
        self.target_state_pub = self.create_publisher(TargetState, "/target_state", 10)
        
        self.annotated_img_pub_ = self.create_publisher(CompressedImage, "/camera/annotated/compressed", 10)
        
        self.id_lock: int = -1

    # todo turn to proper FSM
    def image_callback(self, msg: CompressedImage):
        frame = compressed_imgmsg_to_cv2(msg)
        if frame is None:
            self.get_logger().warning("couldn't extract frame from msg")
            return
            
        result = self.model.track(frame,
            persist=True,
            tracker=self.tracker_config_path,
            verbose=False
        )[0]
        
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
                box_width_px = x2 - x1
                target_cx = (x1 + x2) // 2
                target_cy = (y1 + y2) // 2

                target_state_msg.is_target_visible = True
                
                frame_height, frame_width = frame.shape[0], frame.shape[1]
                cy, cx = frame_height / 2, frame_width / 2
                
                # Estimations:
                depth = (self.drone_width * self.Fx) / box_width_px
                time_to_hit: float = depth / self.velocity_projectile
                
                # #### Accounting for erros
                
                # Muzzle offset
                cy_offset = (self.offset_cam_muzzle/depth) * self.Fy
                cy_corrected = cy + cy_offset
                
                # Predictive Lead
                target_predicted_cx = target_cx
                target_predicted_cy = target_cy
                track = self.get_track()
                if track is not None:
                    Vx_drone, Vy_drone = track.mean[4].item(), track.mean[5].item()
                    target_predicted_cx += (Vx_drone * time_to_hit)
                    target_predicted_cy += (Vy_drone * time_to_hit)
                
                # #### End accounting for errors
                
                
                target_state_msg.norm_err_x = (target_predicted_cx - cx) / frame_width
                target_state_msg.norm_err_y = (target_predicted_cy - cy_corrected) / frame_height
                
                target_state_msg.depth_meters = depth
                
            else:
                target_state_msg.is_target_visible = False
                self.id_lock = -1
        
        self.target_state_pub.publish(target_state_msg)
        
        annotated_img = result.plot()
        annotated_msg = CompressedImage()
        annotated_msg.header = msg.header
        annotated_msg.format = "jpeg"
        annotated_msg.data = cv2.imencode('.jpg', annotated_img)[1].tobytes()
        self.annotated_img_pub_.publish(annotated_msg)

            
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
            
            dist = ((box_cx - cx)**2 + (box_cy - cy)**2)
            if dist < min_dist:
                min_dist = dist
                self.id_lock = int(box.id.item())
                
        return did_find_track
        
    def get_track(self):
            if self.id_lock == -1:
                return None
            if not hasattr(self.model, 'predictor') or not self.model.predictor.trackers:
                return None
                
            tracker = self.model.predictor.trackers[0] 
            active_tracks = getattr(tracker, 'tracked_stracks', [])
            
            for t in active_tracks:
                if t.track_id == self.id_lock:
                    return t
                    
            return None
            
            
    def pick_largest_box(self, result: Results):
        if result.boxes is None:
            return
            
        largest_area = float('-inf')
        largest_box = None
        
        for box in result.boxes:
            xywh = box.xywh[0].tolist() 
            width, height = xywh[2], xywh[3]
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
