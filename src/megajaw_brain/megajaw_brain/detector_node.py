#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import ncnn
from ament_index_python.packages import get_package_share_directory
import cv2
from sensor_msgs.msg import CompressedImage
import os
from megajaw_brain.utils import imgmsg_to_cv2, extract_largest_box
from megajaw_brain import constants
from megajaw_interfaces.msg import TargetControl
from ultralytics import YOLO
import math


def get_depth_gz(width_px):
    fox_x = constants.GZ_CAM_IMG_WIDTH / (2 * math.tan(constants.GZ_CAM_HFOV / 2))
    return (constants.OBJ_WIDTH_METERS * fox_x) / (max(width_px, 1.0))


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.get_logger().info("detector_node Started")

        self.declare_parameter("conf_thresh", 0.5)
        self.conf_thresh = self.get_parameter("conf_thresh").value

        self.declare_parameter("is_sim", True)
        self.is_sim = self.get_parameter("is_sim").value

        self.declare_parameter("debug", True)
        self.debug = self.get_parameter("debug").value

        self.fx = 1497.70202
        self.fy = 1435.16210
        cx = 438.691766
        cy = 918.565271

        self.K = np.array(
            [
                [self.fx, 0.0, cx],
                [0.0, self.fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        self.dist = np.array(
            [0.09600973, -1.13969576, 0.00809251, -0.01848817, 1.54268659],
            dtype=np.float32,
        )
        self.yolo_img_sz = (
            constants.YOLO_IMG_SZ_GZ if self.is_sim else constants.YOLO_IMG_SZ_REAL
        )

        self.net = ncnn.Net()  # type: ignore

        if self.is_sim:
            self.model = YOLO(
                os.path.join(
                    get_package_share_directory("megajaw_brain"),
                    "static",
                    "best_ncnn_model_gz",
                ),
                task="detect",
            )
        else:
            self.model = YOLO(
                os.path.join(
                    get_package_share_directory("megajaw_brain"),
                    "static",
                    "best_ncnn_model_real",
                ),
                task="detect",
            )

        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.image_callback,
            10,
        )

        self.publisher = self.create_publisher(TargetControl, "/target_state", 10)

        # Persistent Target Lock Fields
        self.locked_track_id = None
        self.lost_frame_counter = 0

        self.declare_parameter("max_lost_frames", 30)
        self.max_lost_frames = self.get_parameter("max_lost_frames").value

    def image_callback(self, msg: CompressedImage):
        frame_bgr = imgmsg_to_cv2(msg)
        if not self.is_sim:
            frame_bgr = cv2.undistort(frame_bgr, self.K, self.dist)
            frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_CLOCKWISE)

        h, w = frame_bgr.shape[:2]

        results = self.model.track(
            frame_bgr, persist=True, conf=self.conf_thresh, verbose=False
        )[0]

        boxes = results.boxes
        best_box_xywh = None

        if boxes is not None and boxes.id is not None and len(boxes) > 0:
            ids = boxes.id.cpu().numpy().astype(int).tolist()
            xywh_list = boxes.xywh.cpu().numpy()
            confs = boxes.conf.cpu().numpy()

            # --- Condition A: Locked target is actively present ---
            if self.locked_track_id is not None and self.locked_track_id in ids:
                idx = ids.index(self.locked_track_id)
                best_box_xywh = xywh_list[idx]
                self.lost_frame_counter = 0

            # --- Condition B: Locked target missing (grace buffer) ---
            elif self.locked_track_id is not None and self.locked_track_id not in ids:
                self.lost_frame_counter += 1
                if self.lost_frame_counter > self.max_lost_frames:
                    self.get_logger().warn(
                        f"[TargetLock] Lock DROPPED on ID {self.locked_track_id} "
                        f"after {self.lost_frame_counter} lost frames. Re-acquiring..."
                    )
                    self.locked_track_id = None
                    self.lost_frame_counter = 0
                else:
                    self.get_logger().info(
                        f"[TargetLock] ID {self.locked_track_id} missing "
                        f"({self.lost_frame_counter}/{self.max_lost_frames})"
                    )
                # best_box_xywh stays None while in grace buffer

            # --- Condition C: No active lock — acquire best target ---
            if self.locked_track_id is None:
                best_score = -1.0
                best_idx = -1
                for i in range(len(ids)):
                    bw, bh = float(xywh_list[i][2]), float(xywh_list[i][3])
                    score = bw * bh * float(confs[i])
                    if score > best_score:
                        best_score = score
                        best_idx = i
                if best_idx >= 0:
                    self.locked_track_id = ids[best_idx]
                    best_box_xywh = xywh_list[best_idx]
                    self.lost_frame_counter = 0
                    self.get_logger().info(
                        f"[TargetLock] LOCKED onto ID {self.locked_track_id} "
                        f"(score={best_score:.1f})"
                    )
        else:
            # No detections at all — apply grace buffer logic if locked
            if self.locked_track_id is not None:
                self.lost_frame_counter += 1
                if self.lost_frame_counter > self.max_lost_frames:
                    self.get_logger().warn(
                        f"[TargetLock] Lock DROPPED on ID {self.locked_track_id} "
                        f"(no detections for {self.lost_frame_counter} frames)"
                    )
                    self.locked_track_id = None
                    self.lost_frame_counter = 0

        ctrl_msg = TargetControl()
        ctrl_msg.target_detected = best_box_xywh is not None

        if best_box_xywh is not None:
            orig_h, orig_w = results.orig_shape
            cx_center, _ = orig_w / 2, orig_h / 2
            dx = float(-(best_box_xywh[0] - cx_center) / cx_center)
            ctrl_msg.err_x = dx
            bbox_w_px = float(best_box_xywh[2]) * 2.2
            bbox_h_px = float(best_box_xywh[3]) * 3.3

            if self.is_sim:
                ctrl_msg.depth = get_depth_gz(bbox_w_px)
            else:
                depth_w = (self.fx * constants.OBJ_WIDTH_METERS) / max(bbox_w_px, 1.0)
                depth_h = (self.fy * constants.OBJ_HEIGHT_METERS) / max(bbox_h_px, 1.0)
                ctrl_msg.depth = (depth_w + depth_h) / 2.0

                # self.get_logger().info(f"Depth Width: {depth_w}, DepthHeight: {depth_h}, bbox_w_px: {bbox_w_px}, bbox_h_px: {bbox_h_px}")

        self.publisher.publish(ctrl_msg)

        if self.debug:
            preview_frame = results.plot()

            if best_box_xywh is not None:
                real_cx = int(best_box_xywh[0])
                real_cy = int(best_box_xywh[1])
                cv2.circle(preview_frame, (real_cx, real_cy), 9, (0, 0, 255), -1)

            cv2.imshow("Debug View", preview_frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
