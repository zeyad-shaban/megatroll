#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import ncnn
from ament_index_python.packages import get_package_share_directory
import cv2
from sensor_msgs.msg import CompressedImage
import os
from megajaw_brain.utils import imgmsg_to_cv2, draw_detections, extract_largest_box
from megajaw_brain import constants
from megajaw_interfaces.msg import TargetControl
import math


def get_depth_gz(width_px):
    fox_x = constants.GZ_CAM_IMG_WIDTH / (2 * math.tan(constants.GZ_CAM_HFOV / 2))
    return (constants.OBJ_WIDTH_METERS * fox_x) / (max(width_px, 1.0))


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.get_logger().info(f"detector_node Started")

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
            self.net.load_param(
                os.path.join(
                    get_package_share_directory("megajaw_brain"),
                    "static",
                    "best_ncnn_model_gz",
                    "model.ncnn.param",
                )
            )
            self.net.load_model(
                os.path.join(
                    get_package_share_directory("megajaw_brain"),
                    "static",
                    "best_ncnn_model_gz",
                    "model.ncnn.bin",
                )
            )
        else:
            self.net.load_param(
                os.path.join(
                    get_package_share_directory("megajaw_brain"),
                    "static",
                    "best_ncnn_model_real",
                    "model.ncnn.param",
                )
            )
            self.net.load_model(
                os.path.join(
                    get_package_share_directory("megajaw_brain"),
                    "static",
                    "best_ncnn_model_real",
                    "model.ncnn.bin",
                )
            )

        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.image_callback,
            10,
        )

        self.publisher = self.create_publisher(TargetControl, "/target_state", 10)

    def image_callback(self, msg: CompressedImage):
        frame_bgr = imgmsg_to_cv2(msg)
        if not self.is_sim:
            frame_bgr = cv2.undistort(frame_bgr, self.K, self.dist)

        frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_CLOCKWISE)
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w = frame_rgb.shape[:2]

        mat = ncnn.Mat.from_pixels_resize(
            frame_rgb,
            ncnn.Mat.PixelType.PIXEL_RGB,
            w,
            h,
            self.yolo_img_sz,
            self.yolo_img_sz,
        )

        mean_vals = (0.0, 0.0, 0.0)
        norm_vals = (1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0)
        mat.substract_mean_normalize(mean_vals, norm_vals)

        ex = self.net.create_extractor()
        ex.input("in0", mat)

        ret, out = ex.extract("out0")
        out = np.array(out)  # 5 (cx, cy, nw, nh, class_score) x n_anchors

        if ret == -1:
            self.get_logger().error("Failed to extract output from the model ret = -1")

        largest_box, largest_box_raw = extract_largest_box(out, conf_thresh=self.conf_thresh)

        ctrl_msg = TargetControl()
        ctrl_msg.target_detected = largest_box is not None

        if largest_box is not None:
            cx, cy = self.yolo_img_sz // 2, self.yolo_img_sz // 2
            dx = -(largest_box["cx"] - cx) / (self.yolo_img_sz // 2)

            ctrl_msg.err_x = dx
            bbox_w_px = float(largest_box["w"]) * (w / self.yolo_img_sz) * 2.2
            bbox_h_px = float(largest_box["h"]) * (h / self.yolo_img_sz) * 3.3

            if self.is_sim:
                ctrl_msg.depth = get_depth_gz(bbox_w_px)
            else:
                depth_w = (self.fx * constants.OBJ_WIDTH_METERS) / max(bbox_w_px, 1.0)
                depth_h = (self.fy * constants.OBJ_HEIGHT_METERS) / max(bbox_h_px, 1.0)
                self.get_logger().info(
                    f"Depth Width: {depth_w}, DepthHeight: {depth_h}, bbox_w_px: {bbox_w_px}, bbox_h_px: {bbox_h_px}"
                )
                ctrl_msg.depth = (depth_w + depth_h) / 2

        self.publisher.publish(ctrl_msg)

        if self.debug:
            detected_out = np.reshape(largest_box_raw, (5,1)) if largest_box_raw is not None else np.empty((5,0))
            preview_frame = draw_detections(frame_bgr, self.yolo_img_sz, detected_out, conf_thresh=self.conf_thresh)

            if largest_box is not None:
                frame_h, frame_w = preview_frame.shape[:2]
                x_scale = frame_w / self.yolo_img_sz
                y_scale = frame_h / self.yolo_img_sz

                real_cx = int(largest_box["cx"] * x_scale)
                real_cy = int(largest_box["cy"] * y_scale)

                cv2.circle(preview_frame, (real_cx, real_cy), 9, (0, 0, 255), 2)

            cv2.imshow("Debug View", preview_frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
