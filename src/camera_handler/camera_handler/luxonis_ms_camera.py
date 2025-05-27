#!/usr/bin/env python3
"""
Stand-alone OAK-D-Lite recorder with calibration + rectification into the
COLOR camera’s epipolar frame.

* DepthAI → CV2 VideoWriter (loss-less FFV1) – rgb, left, right (rectified)
* timestamps.csv  : stream,frame_idx,ts,x,y,z
* coords.csv      : ts,x,y,z      (every R82 UART message)
* Publishes one raster command on /sequencer at launch
* Uses oak_calib.yaml from camera_handler/config for intrinsics/extrinsics
* Locks AE/AWB and applies manual exposure/ISO/WB on all cameras
"""

import os, csv, threading, time
from typing import Tuple, List
from pathlib import Path

import cv2, depthai as dai, numpy as np, yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# ───────────────────────────── Load calibration ─────────────────────────────
cfg_dir   = get_package_share_directory("camera_handler")
calib_fn  = os.path.join(cfg_dir, "config", "oak_calib.yaml")
calib     = yaml.safe_load(Path(calib_fn).read_text())

# Intrinsics
KC, DC = np.array(calib["KC"]), np.array(calib["DC"])
KL, DL = np.array(calib["KL"]), np.array(calib["DL"])
KR, DR = np.array(calib["KR"]), np.array(calib["DR"])

# Extrinsics
R_lr, T_lr = np.array(calib["R"]),  np.array(calib["T"])   # left→right
R_cl, T_cl = np.array(calib["Rc"]), np.array(calib["Tc"])  # color→left

# build color→right by composition:  X_r = R_lr * (Rc * X_c + Tc) + T_lr
R_cr = R_lr.dot(R_cl)
T_cr = R_lr.dot(T_cl) + T_lr

# image size
W, H = 1920, 1200

# ─────────── stereoRectify: COLOR ↔ LEFT ───────────────────────────────
RL_cl, RR_cl, PL_cl, PR_cl, _, _, _ = cv2.stereoRectify(
    KC, DC, KL, DL, (W, H), R_cl, T_cl,
    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
)
# maps for LEFT → color epipolar
mapL1, mapL2 = cv2.initUndistortRectifyMap(KL, DL, RR_cl, PR_cl, (W, H), cv2.CV_16SC2)

# ─────────── stereoRectify: COLOR ↔ RIGHT ──────────────────────────────
RL_cr, RR_cr, PL_cr, PR_cr, _, _, _ = cv2.stereoRectify(
    KC, DC, KR, DR, (W, H), R_cr, T_cr,
    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
)
# maps for RIGHT → color epipolar
mapR1, mapR2 = cv2.initUndistortRectifyMap(KR, DR, RR_cr, PR_cr, (W, H), cv2.CV_16SC2)


# ───────────────────────────── ROS helper node ────────────────────────────
class AuxNode(Node):
    """Subscribes to /uart_receive, publishes raster once, stores coords."""
    def __init__(self):
        super().__init__("oak_aux")
        self._x = self._y = self._z = 0.0
        self._lock = threading.Lock()

        # CSV for robot coords
        self._fh = open("coords.csv", "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(["ts", "x", "y", "z"])

        self.create_subscription(String, "uart_receive", self._uart_cb, 10)

        # publish raster sequence once
        self._pub = self.create_publisher(String, "sequencer", 10)
        self._publish_raster()

    def _publish_raster(self):
        try:
            mfn = os.path.join(
                get_package_share_directory("map_handler"),
                "config", "active_map.yaml"
            )
            m = yaml.safe_load(Path(mfn).read_text())
        except Exception as e:
            self.get_logger().error(f"Cannot load active_map.yaml: {e}")
            return

        plants = m["plant_details"]["plants"]
        xs = {p["position"]["x"] for p in plants.values()}
        if not xs:
            self.get_logger().warn("No plants → no raster")
            return

        y_min, y_max = 0, m["map_reference"]["y_len"]
        cmds: List[str] = []
        for i, x in enumerate(sorted(xs)):
            y0, y1 = (y_min, y_max) if i % 2 == 0 else (y_max, y_min)
            cmds += [
                f"CS_COL_{i:03}\n",
                f"{x} {y0} 0.0 200\n",
                f"CS_COL_{i:03}\n",
                f"{x} {y1} 0.0 200\n",
            ]
        msg = String(); msg.data = "".join(cmds).rstrip("\n")
        self._pub.publish(msg)
        self.get_logger().info("Raster sequence published")

    def _uart_cb(self, msg: String):
        parts = msg.data.split()
        if parts and parts[0] == "R82" and len(parts) >= 4:
            try:
                x = float(parts[1][1:])
                y = float(parts[2][1:])
                z = float(parts[3][1:])
            except ValueError:
                return
            with self._lock:
                self._x, self._y, self._z = x, y, z
            t0 = time.time()
            self._csv.writerow([f"{t0:.6f}", f"{x:.2f}", f"{y:.2f}", f"{z:.2f}"])
            self._fh.flush()

    def coords(self) -> Tuple[float,float,float]:
        with self._lock:
            return self._x, self._y, self._z

    def close(self):
        self._fh.close()


# ───────────────────────────── DepthAI pipeline ───────────────────────────
def build_pipeline() -> dai.Pipeline():
    p = dai.Pipeline()

    def add_color(name, socket, iso, exp_us, wb_k):
        cam = p.createColorCamera()
        cam.setBoardSocket(socket)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(10)
        ctrl = cam.initialControl
        ctrl.setAutoExposureLock(True)
        ctrl.setManualExposure(exp_us, iso)
        ctrl.setAutoWhiteBalanceLock(True)
        ctrl.setManualWhiteBalance(wb_k)
        xout = p.createXLinkOut(); xout.setStreamName(name)
        cam.isp.link(xout.input)

    # your per-camera settings from the other script
    add_color("left",  dai.CameraBoardSocket.CAM_B, iso=500, exp_us=10000, wb_k=4500)
    add_color("rgb",   dai.CameraBoardSocket.CAM_A, iso=200, exp_us=4000,  wb_k=4500)
    add_color("right", dai.CameraBoardSocket.CAM_C, iso=500, exp_us=7000,  wb_k=4500)

    return p


# ───────────────────────────── Main script ──────────────────────────────
def main():
    rclpy.init()
    aux   = AuxNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(aux)
    t = threading.Thread(target=exec_.spin, daemon=True)
    t.start()

    pipeline = build_pipeline()
    device   = dai.Device(pipeline)
    q_rgb    = device.getOutputQueue("rgb",   maxSize=4, blocking=False)
    q_left   = device.getOutputQueue("left",  maxSize=4, blocking=False)
    q_right  = device.getOutputQueue("right", maxSize=4, blocking=False)

    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    fps    = 10.0
    writers = {
        "rgb":   cv2.VideoWriter("rgb_video.mkv",   fourcc, fps, (W, H), True),
        "left":  cv2.VideoWriter("left_video.mkv",  fourcc, fps, (W, H), False),
        "right": cv2.VideoWriter("right_video.mkv", fourcc, fps, (W, H), False),
    }

    ts_fh  = open("timestamps.csv","w",newline="")
    ts_csv = csv.writer(ts_fh)
    ts_csv.writerow(["stream","frame_idx","ts","x","y","z"])
    idx = {"rgb":0, "left":0, "right":0}

    print("Recording... Ctrl-C to stop.")
    try:
        while rclpy.ok():
            for name, q in (("rgb",q_rgb),("left",q_left),("right",q_right)):
                pkt = q.tryGet()
                if not pkt:
                    continue
                frame = pkt.getCvFrame()
                # rectify only left/right into the COLOR plane:
                if name == "left":
                    frame = cv2.remap(frame, mapL1, mapL2, cv2.INTER_LINEAR)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                elif name == "right":
                    frame = cv2.remap(frame, mapR1, mapR2, cv2.INTER_LINEAR)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                writers[name].write(frame)

                t0 = time.time()
                x,y,z = aux.coords()
                ts_csv.writerow([
                    name,
                    idx[name],
                    f"{t0:.6f}",
                    f"{x:.2f}",
                    f"{y:.2f}",
                    f"{z:.2f}"
                ])
                idx[name] += 1

    except KeyboardInterrupt:
        pass

    # cleanup
    for w in writers.values(): w.release()
    ts_fh.close()
    aux.close()
    exec_.shutdown()
    rclpy.shutdown()
    device.close()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
