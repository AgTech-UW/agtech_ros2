#!/bin/bash
set -e

echo "=== AgTech ROS 2 container bootstrap ==="

echo "Sourcing ROS 2 in bashrc..."
grep -q "source /opt/ros/humble/setup.bash" /root/.bashrc || \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

echo "Setting Cyclone DDS..."
grep -q "RMW_IMPLEMENTATION" /root/.bashrc || \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc

echo "Installing ROS packages..."
apt-get update
apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-apriltag-ros \
    ros-humble-image-proc \
    ros-humble-camera-calibration \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-topic-tools \
    v4l-utils

echo "Restoring camera calibration..."
mkdir -p /root/.ros/camera_info
if [ -f /workspaces/agtech_ros2/config/default_cam.yaml ]; then
    cp /workspaces/agtech_ros2/config/default_cam.yaml /root/.ros/camera_info/
    echo "  Calibration installed."
else
    echo "  WARNING: no calibration file found at config/default_cam.yaml"
    echo "  Students: you'll need to run the checkerboard calibration yourself."
fi

echo "Mystery folder (easter egg)..."
mkdir -p /root/mystery_folder/level_1
echo 'CODE: KERNEL_OF_TRUTH' > /root/mystery_folder/level_1/.hidden_payload.txt

echo ""
echo "=== Bootstrap complete! ==="
echo "Open a NEW terminal to pick up bashrc changes."
echo "Then launch: ros2 launch launch/indoor_gps.launch.py"
