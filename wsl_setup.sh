#!/bin/bash
# WSL Setup Script for Unitree Go2
# Run this INSIDE WSL (not Windows terminal)
#
# First install WSL from Windows:
#   wsl --install
# Then open WSL and run:
#   cd /mnt/c/robot_dog
#   bash wsl_setup.sh

set -e

echo "========================================="
echo "  Go2 WSL Setup"
echo "========================================="

# Install system dependencies
echo "[1/5] Installing system packages..."
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential cmake

# Create venv
echo "[2/5] Creating virtual environment..."
python3 -m venv go2env_wsl
source go2env_wsl/bin/activate

# Install Python packages
echo "[3/5] Installing Python packages..."
pip install --upgrade pip
pip install numpy opencv-python

# Install SDK with the correct cyclonedds version
echo "[4/5] Installing Unitree SDK..."
cd unitree_sdk2_python
git checkout setup.py  # undo the Windows cyclonedds edit
pip install -e .
cd ..

# Verify
echo "[5/5] Verifying imports..."
python3 -c "
import cv2
import numpy
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient
print('All imports OK')
print('  numpy:', numpy.__version__)
print('  opencv:', cv2.__version__)
"

echo ""
echo "========================================="
echo "  Setup complete!"
echo "========================================="
echo ""
echo "To use:"
echo "  cd /mnt/c/robot_dog"
echo "  source go2env_wsl/bin/activate"
echo "  python go2_starter.py       # basic commands"
echo "  python go2_drive.py         # WASD + camera feeds"
echo "  python go2_camera.py        # camera only"
echo "  python go2_sensors.py       # camera + lidar"
echo ""
echo "NOTE: If cv2.imshow doesn't work in WSL, install an X server"
echo "on Windows (VcXsrv or WSLg if on Win11) and set:"
echo "  export DISPLAY=:0"
