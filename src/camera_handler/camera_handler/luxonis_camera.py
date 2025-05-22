#!/usr/bin/env python3
"""Minimal OAK‑D‑PoE ROS 2 driver that mimics the plain‑Python example structure
while publishing **rgb** and **depth** topics.

* Keeps the original `build_pipeline()` and `main()` top‑level flow.
* Loads `<camera_name>.yaml` from *camera_handler/config/* for limits & IP.
* Publishes
  * `<camera_name>_rgb`  – BGR8 frames @ 4k, 25 fps.
  * `<camera_name>_depth` – 8‑bit visual depth map derived from stereo disparity.
"""

import os
import time
import threading
from typing import Optional

import depthai as dai
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import yaml


# ────────────────────────── Pipeline helper ──────────────────────────

def build_pipeline(mono_res: str, color_res: str) -> dai.Pipeline:
    """Construct a DepthAI pipeline with RGB‑ISP video and Stereo disparity."""
    p = dai.Pipeline()

    # RGB @1080p (video output ⇒ full frame, rather than the smaller preview)
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(
        {
            "THE_4_K": dai.ColorCameraProperties.SensorResolution.THE_4_K,
            "THE_1080_P": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "THE_1200_P": dai.ColorCameraProperties.SensorResolution.THE_1200_P,
            "THE_800_P": dai.ColorCameraProperties.SensorResolution.THE_800_P,
        }[color_res]
    )
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(25)

    xout_rgb = p.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam.isp.link(xout_rgb.input)

    # Mono cameras
    mono_enum = {
        "THE_400_P": dai.MonoCameraProperties.SensorResolution.THE_400_P,
        "THE_720_P": dai.MonoCameraProperties.SensorResolution.THE_720_P,
        "THE_1200_P": dai.MonoCameraProperties.SensorResolution.THE_1200_P,
    }[mono_res]

    mono_l = p.create(dai.node.MonoCamera)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_l.setResolution(mono_enum)
    mono_l.setFps(25)

    mono_r = p.create(dai.node.MonoCamera)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_r.setResolution(mono_enum)
    mono_r.setFps(25)

    # Stereo depth
    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(False)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(False)

    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    xout_stereo = p.create(dai.node.XLinkOut)
    xout_stereo.setStreamName("disp")
    stereo.disparity.link(xout_stereo.input)

    return p


# ───────────────────────────── Driver node ───────────────────────────

class OakPoeDriver(Node):
    def __init__(self):
        super().__init__("luxonis_driver")

        # ─── Parameters ───
        self.declare_parameter("camera_name", "luxonis_lite")
        self.camera_name = self.get_parameter("camera_name").get_parameter_value().string_value

        # Load YAML config
        self.load_config()

        # ─── DepthAI ― pipeline + device ───
        pipeline = build_pipeline(self.mono_res_, self.color_res_)
        self.device = dai.Device(pipeline)

        # Pre‑compute focal/baseline for depth conversion
        calib = self.device.readCalibration()
        fx_l = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)[0][0]
        fx_r = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C)[0][0]
        self.focal_px = (fx_l + fx_r) / 2
        self.baseline_mm = calib.getBaselineDistance(
            dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
        )

        # ─── ROS I/O ───
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, f"rgb", 10)
        self.depth_pub = self.create_publisher(Image, f"depth", 10)

        # DepthAI queues
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.q_disp = self.device.getOutputQueue("disp",     maxSize=4, blocking=False)

        # Packet polling thread
        self.running = True
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()
        self.get_logger().info("OAK‑D‑PoE driver running")

    # ───────────────────────── Config loader ─────────────────────────

    def load_config(self):
        pkg_dir = get_package_share_directory("camera_handler")
        cfg_path = os.path.join(pkg_dir, "config", f"{self.camera_name}_config.yaml")
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(cfg_path)
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)
        cc = cfg["camera_constants"]
        self.DEPTH_MIN_VAL = cc["DEPTH_MIN_VAL"]
        self.DEPTH_MAX_VAL = cc["DEPTH_MAX_VAL"]
        self.mono_res_ = "THE_400_P"  # cc["MONO_RESOLUTION"]
        self.color_res_ = "THE_1080_P"

    # ───────────────────────── Packet loop ──────────────────────────

    def loop(self):
        while self.running and rclpy.ok():
            # RGB
            pkt = self.q_rgb.tryGet()
            if pkt is not None:
                frame = pkt.getCvFrame()
                self.publish_rgb(frame)
            # Disparity
            pkt = self.q_disp.tryGet()
            if pkt is not None:
                depth = self.disparity_to_depth(pkt.getFrame())
                self.publish_depth(depth)
            time.sleep(0.002)

    # ───────────────────── Publish helpers ───────────────────────────

    def publish_rgb(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.rgb_pub.publish(msg)

    def publish_depth(self, depth_frame):
        msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono8")
        self.depth_pub.publish(msg)

    # ───────────────────── Depth conversion ─────────────────────────

    def disparity_to_depth(self, disp):
        disp[disp == 0] = 0.01
        depth = (self.focal_px * self.baseline_mm) / disp
        depth[(depth >= self.DEPTH_MAX_VAL) | (depth <= self.DEPTH_MIN_VAL)] = 0
        depth_8u = np.clip(depth / self.DEPTH_MAX_VAL * 255.0, 0, 255).astype(np.uint8)
        return depth_8u

    # ───────────────────── Shutdown cleanly ─────────────────────────

    def destroy_node(self):
        self.running = False
        if self.th.is_alive():
            self.th.join()
        self.device.close()
        super().destroy_node()


# ────────────────────────────── main() ──────────────────────────────


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = OakPoeDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
