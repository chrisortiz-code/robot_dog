"""
Unitree Go2 Sensor Viewer
--------------------------
Shows the front camera feed and a top-down lidar plot side by side.

Press ESC to quit. Press S to save a screenshot.

Usage:
    python go2_sensors.py
    python go2_sensors.py eth0   # specify network interface
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
)
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


# ---------------------------------------------------------------------------
# Lidar subscriber — runs in background, stores latest point cloud
# ---------------------------------------------------------------------------
class LidarReceiver:
    def __init__(self):
        self.points = None
        self.lock = threading.Lock()

    def start(self):
        sub = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        sub.Init(self._callback, 10)

    def _callback(self, msg: PointCloud2_):
        """Parse x,y from the raw point cloud bytes."""
        data = bytes(msg.data)
        step = msg.point_step
        count = msg.width * msg.height
        if count == 0 or step == 0:
            return

        # Extract x, y floats from each point (first 8 bytes = 2 floats)
        pts = []
        for i in range(count):
            offset = i * step
            if offset + 8 > len(data):
                break
            x, y = struct.unpack_from("<ff", data, offset)
            # Filter out garbage / zero points
            dist = x * x + y * y
            if 0.01 < dist < 400:  # between 0.1m and 20m
                pts.append((x, y))

        if pts:
            with self.lock:
                self.points = np.array(pts, dtype=np.float32)

    def get_points(self):
        with self.lock:
            return self.points.copy() if self.points is not None else None


def render_lidar_topdown(points, size=480):
    """Render a top-down 2D image of lidar points."""
    img = np.zeros((size, size, 3), dtype=np.uint8)

    if points is None or len(points) == 0:
        cv2.putText(img, "No lidar data", (size // 4, size // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return img

    # Scale: 1 meter = some pixels. Auto-scale to fit.
    max_range = max(np.abs(points).max(), 1.0)
    scale = (size // 2 - 20) / max_range

    center = size // 2

    # Draw points
    for x, y in points:
        px = int(center + y * scale)  # y is lateral
        py = int(center - x * scale)  # x is forward (up in image)
        if 0 <= px < size and 0 <= py < size:
            cv2.circle(img, (px, py), 1, (0, 255, 0), -1)

    # Draw robot position
    cv2.circle(img, (center, center), 5, (0, 0, 255), -1)
    cv2.putText(img, "LIDAR (top-down)", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(img, f"range: {max_range:.1f}m", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(img, f"points: {len(points)}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return img


def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # --- Start camera ---
    video = VideoClient()
    video.SetTimeout(3.0)
    video.Init()

    # --- Start lidar subscriber ---
    lidar = LidarReceiver()
    lidar.start()

    # Turn lidar on
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    pub = ChannelPublisher("rt/utlidar/switch", String_)
    pub.Init()
    cmd = String_()
    cmd.data = "ON"
    pub.Write(cmd)

    print("Sensor viewer started. Press ESC to quit, S to screenshot.")

    shot_count = 0
    lidar_size = 480

    while True:
        # --- Camera frame ---
        code, data = video.GetImageSample()
        if code == 0 and data:
            img_data = np.frombuffer(bytes(data), dtype=np.uint8)
            cam_img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        else:
            cam_img = np.zeros((lidar_size, 640, 3), dtype=np.uint8)
            cv2.putText(cam_img, f"No camera (code: {code})", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Resize camera to match lidar panel height
        if cam_img is not None:
            h, w = cam_img.shape[:2]
            scale = lidar_size / h
            cam_img = cv2.resize(cam_img, (int(w * scale), lidar_size))

        # --- Lidar frame ---
        lidar_img = render_lidar_topdown(lidar.get_points(), lidar_size)

        # --- Combine side by side ---
        combined = np.hstack([cam_img, lidar_img])

        cv2.imshow("Go2 Sensors", combined)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            break
        elif key == ord("s"):
            filename = f"go2_sensors_{shot_count}.jpg"
            cv2.imwrite(filename, combined)
            print(f"Saved {filename}")
            shot_count += 1

    cv2.destroyAllWindows()
    print("Viewer stopped.")


if __name__ == "__main__":
    main()
