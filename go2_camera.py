"""
Unitree Go2 Camera Feed
------------------------
Streams the front camera feed in a window.
Press ESC to quit. Press S to save a screenshot.

Usage:
    python go2_camera.py
    python go2_camera.py eth0   # specify network interface
"""

import sys
import numpy as np
import cv2
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    print("Starting camera feed... Press ESC to quit, S to save a screenshot.")

    shot_count = 0
    code, data = client.GetImageSample()

    if code != 0:
        print(f"Failed to get image from camera. Error code: {code}")
        print("Check that your PC can reach the dog (ping 192.168.123.161)")
        return

    while code == 0:
        code, data = client.GetImageSample()
        if code != 0:
            print(f"Lost camera feed. Error code: {code}")
            break

        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        if image is None:
            continue

        cv2.imshow("Go2 Front Camera", image)

        key = cv2.waitKey(20) & 0xFF
        if key == 27:  # ESC
            break
        elif key == ord("s"):
            filename = f"go2_screenshot_{shot_count}.jpg"
            cv2.imwrite(filename, image)
            print(f"Saved {filename}")
            shot_count += 1

    cv2.destroyAllWindows()
    print("Camera feed stopped.")


if __name__ == "__main__":
    main()
