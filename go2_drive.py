"""
Unitree Go2 Drive Mode
-----------------------
WASD controls + live camera/lidar feeds in one window.

Controls (click the window first so it captures keys):
    W / S   — forward / backward
    A / D   — rotate left / right
    SPACE   — stop movement
    1       — stand up
    2       — sit down
    3       — recovery stand
    ESC     — stop and quit
    P       — screenshot

Usage:
    python go2_drive.py
    python go2_drive.py eth0   # specify network interface
"""

import sys
import struct
import threading
import time
import numpy as np
import cv2
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
    ChannelPublisher,
)
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


# ---- Settings ----
MOVE_SPEED = 0.4       # m/s forward/backward
ROTATE_SPEED = 0.6     # rad/s turning
PANEL_HEIGHT = 400      # pixel height for each feed panel
DEPTH_CAM_INDEX = 1     # USB depth cam index (try 0, 1, 2 if wrong)


# ---------------------------------------------------------------------------
# Lidar
# ---------------------------------------------------------------------------
class LidarReceiver:
    def __init__(self):
        self.points = None
        self.lock = threading.Lock()

    def start(self):
        sub = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        sub.Init(self._callback, 10)

    def _callback(self, msg: PointCloud2_):
        data = bytes(msg.data)
        step = msg.point_step
        count = msg.width * msg.height
        if count == 0 or step == 0:
            return
        pts = []
        for i in range(count):
            offset = i * step
            if offset + 8 > len(data):
                break
            x, y = struct.unpack_from("<ff", data, offset)
            dist = x * x + y * y
            if 0.01 < dist < 400:
                pts.append((x, y))
        if pts:
            with self.lock:
                self.points = np.array(pts, dtype=np.float32)

    def get_points(self):
        with self.lock:
            return self.points.copy() if self.points is not None else None


def render_lidar(points, size):
    img = np.zeros((size, size, 3), dtype=np.uint8)
    if points is None or len(points) == 0:
        cv2.putText(img, "No lidar data", (10, size // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return img

    max_range = max(np.abs(points).max(), 1.0)
    scale = (size // 2 - 20) / max_range
    c = size // 2

    for x, y in points:
        px = int(c + y * scale)
        py = int(c - x * scale)
        if 0 <= px < size and 0 <= py < size:
            cv2.circle(img, (px, py), 1, (0, 255, 0), -1)

    cv2.circle(img, (c, c), 4, (0, 0, 255), -1)
    cv2.putText(img, "LIDAR", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


# ---------------------------------------------------------------------------
# Depth camera (USB-C) — tries to open as a regular USB camera via OpenCV
# ---------------------------------------------------------------------------
class DepthCam:
    def __init__(self, index=DEPTH_CAM_INDEX):
        self.cap = cv2.VideoCapture(index)
        if self.cap.isOpened():
            print(f"Depth camera opened on index {index}")
        else:
            print(f"Could not open depth camera on index {index} — will show placeholder")

    def read(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None

    def release(self):
        if self.cap:
            self.cap.release()


# ---------------------------------------------------------------------------
# HUD overlay
# ---------------------------------------------------------------------------
def draw_hud(img, vx, vz):
    h, w = img.shape[:2]
    # Movement indicator
    if vx > 0:
        direction = "FWD"
    elif vx < 0:
        direction = "REV"
    elif vz > 0:
        direction = "< ROT"
    elif vz < 0:
        direction = "ROT >"
    else:
        direction = "STOP"

    cv2.putText(img, direction, (w - 100, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(img, "WASD:move  SPACE:stop  ESC:quit", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # --- Sport client (movement) ---
    sport = SportClient()
    sport.SetTimeout(10.0)
    sport.Init()

    # --- Front camera ---
    video = VideoClient()
    video.SetTimeout(3.0)
    video.Init()

    # --- Depth camera (USB) ---
    depth_cam = DepthCam(DEPTH_CAM_INDEX)

    # --- Lidar ---
    lidar = LidarReceiver()
    lidar.start()
    pub = ChannelPublisher("rt/utlidar/switch", String_)
    pub.Init()
    lidar_cmd = String_()
    lidar_cmd.data = "ON"
    pub.Write(lidar_cmd)

    print("=" * 50)
    print("  Go2 Drive Mode")
    print("  W/S = forward/back  A/D = rotate  SPACE = stop")
    print("  1 = stand up  2 = sit down  3 = recovery")
    print("  ESC = quit    P = screenshot")
    print("=" * 50)

    current_vx = 0.0
    current_vz = 0.0
    shot_count = 0

    while True:
        # ---- Front camera ----
        code, data = video.GetImageSample()
        if code == 0 and data:
            arr = np.frombuffer(bytes(data), dtype=np.uint8)
            front_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        else:
            front_img = None

        if front_img is not None:
            fh, fw = front_img.shape[:2]
            s = PANEL_HEIGHT / fh
            front_img = cv2.resize(front_img, (int(fw * s), PANEL_HEIGHT))
        else:
            front_img = np.zeros((PANEL_HEIGHT, 530, 3), dtype=np.uint8)
            cv2.putText(front_img, "No front cam", (20, PANEL_HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(front_img, "FRONT", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # ---- Depth camera ----
        depth_frame = depth_cam.read()
        if depth_frame is not None:
            dh, dw = depth_frame.shape[:2]
            s = PANEL_HEIGHT / dh
            depth_img = cv2.resize(depth_frame, (int(dw * s), PANEL_HEIGHT))
        else:
            depth_img = np.zeros((PANEL_HEIGHT, 530, 3), dtype=np.uint8)
            cv2.putText(depth_img, "No depth cam", (20, PANEL_HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(depth_img, "DEPTH", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # ---- Lidar ----
        lidar_img = render_lidar(lidar.get_points(), PANEL_HEIGHT)

        # ---- Combine: [front | depth | lidar] ----
        combined = np.hstack([front_img, depth_img, lidar_img])
        draw_hud(combined, current_vx, current_vz)

        cv2.imshow("Go2 Drive", combined)

        # ---- Key handling ----
        key = cv2.waitKey(50) & 0xFF

        if key == 27:  # ESC
            sport.StopMove()
            break
        elif key == ord("w"):
            current_vx = MOVE_SPEED
            current_vz = 0.0
            sport.Move(current_vx, 0, current_vz)
        elif key == ord("s"):
            current_vx = -MOVE_SPEED
            current_vz = 0.0
            sport.Move(current_vx, 0, current_vz)
        elif key == ord("a"):
            current_vx = 0.0
            current_vz = ROTATE_SPEED
            sport.Move(current_vx, 0, current_vz)
        elif key == ord("d"):
            current_vx = 0.0
            current_vz = -ROTATE_SPEED
            sport.Move(current_vx, 0, current_vz)
        elif key == ord(" "):
            current_vx = 0.0
            current_vz = 0.0
            sport.StopMove()
        elif key == ord("1"):
            sport.StandUp()
        elif key == ord("2"):
            sport.StandDown()
        elif key == ord("3"):
            sport.RecoveryStand()
        elif key == ord("p"):
            fname = f"go2_drive_{shot_count}.jpg"
            cv2.imwrite(fname, combined)
            print(f"Saved {fname}")
            shot_count += 1

    depth_cam.release()
    cv2.destroyAllWindows()
    print("Drive mode stopped.")


if __name__ == "__main__":
    main()
