#!/bin/bash
# =============================================================================
#   GOD NODE LAUNCHER v2 — One-Script Startup
#   AgTech ROS Course | PHYS-4000 / AG-3000
# =============================================================================
#
#   Launches the full ceiling camera pipeline + tractor control.
#   Fixes: longer camera warmup, pre-stream flush, auto-detect device.
#
#   Usage:
#     bash god_node_launcher.sh          # interactive menu
#     bash god_node_launcher.sh 1        # skip menu, launch full pipeline
#     bash god_node_launcher.sh 2        # skip menu, camera only
#     bash god_node_launcher.sh 3        # skip menu, simulation only
#
# =============================================================================

# =====================================================================
# DEFAULTS (edit these to match your setup)
# =====================================================================
VIDEO_DEVICE=""  # auto-detected
IMAGE_WIDTH=1920
IMAGE_HEIGHT=1080
FRAMERATE=120.0
PIXEL_FORMAT="mjpeg2rgb"
CAMERA_NAME="default_cam"
CAMERA_CALIB="file:///root/.ros/camera_info/default_cam.yaml"
ARENA_HEIGHT=2.58

EXPOSURE=80
GAIN=10
BRIGHTNESS=1

TAG_FAMILY="36h11"
TAG_SIZE=0.150
DECIMATE=2.0
THREADS=4

ROS_DOMAIN=1
CYCLONE_IFACE="wlp130s0"

WORKSPACE="/workspaces/agtech_ros2"

# Timing (seconds) — increase these if camera crashes on startup
CAMERA_WARMUP=8
POST_TUNE_WAIT=3
POST_APRILTAG_WAIT=3
POST_TF_WAIT=2
POST_GOD_WAIT=2

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# PID tracking
PIDS=()

# =====================================================================
# FUNCTIONS
# =====================================================================

print_banner() {
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║         GOD NODE LAUNCHER v2 — AgTech ROS              ║"
    echo "║         Ceiling Camera Pipeline Startup                ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_config() {
    echo -e "${BOLD}Current Configuration:${NC}"
    echo -e "  Camera:        ${GREEN}${VIDEO_DEVICE:-auto-detect}${NC} @ ${IMAGE_WIDTH}x${IMAGE_HEIGHT} ${FRAMERATE}fps"
    echo -e "  Exposure:      ${GREEN}${EXPOSURE}${NC}  Gain: ${GREEN}${GAIN}${NC}  Brightness: ${GREEN}${BRIGHTNESS}${NC}"
    echo -e "  Tag:           ${GREEN}${TAG_FAMILY}${NC}  Size: ${GREEN}${TAG_SIZE}m${NC}"
    echo -e "  Arena Height:  ${GREEN}${ARENA_HEIGHT}m${NC}"
    echo -e "  Domain ID:     ${GREEN}${ROS_DOMAIN}${NC}"
    echo -e "  Network:       ${GREEN}${CYCLONE_IFACE}${NC}"
    echo -e "  Workspace:     ${GREEN}${WORKSPACE}${NC}"
    echo -e "  Camera warmup: ${GREEN}${CAMERA_WARMUP}s${NC}"
    echo ""
}

detect_camera() {
    echo -e "${YELLOW}Detecting camera...${NC}"
    if ! command -v v4l2-ctl &> /dev/null; then
        echo -e "  ${RED}v4l2-ctl not found. Install v4l-utils first.${NC}"
        return 1
    fi

    local cam_dev
    cam_dev=$(v4l2-ctl --list-devices 2>/dev/null | grep -A1 "See3CAM" | grep "/dev/video" | head -1 | tr -d '[:space:]')

    if [ -n "$cam_dev" ]; then
        VIDEO_DEVICE="$cam_dev"
        echo -e "  Found See3CAM: ${GREEN}${VIDEO_DEVICE}${NC}"
    else
        echo -e "  ${RED}See3CAM not found!${NC}"
        echo -e "  Available devices:"
        v4l2-ctl --list-devices 2>/dev/null || echo "    (none)"
        echo ""
        read -p "  Enter device manually (e.g. /dev/video2): " VIDEO_DEVICE
        if [ -z "$VIDEO_DEVICE" ]; then
            echo -e "  ${RED}No device specified. Aborting.${NC}"
            return 1
        fi
    fi
    return 0
}

ensure_calibration() {
    if [ ! -f /root/.ros/camera_info/default_cam.yaml ]; then
        echo -e "${YELLOW}Creating camera calibration file...${NC}"
        mkdir -p ~/.ros/camera_info
        cat << 'CALEOF' > ~/.ros/camera_info/default_cam.yaml
image_width: 1920
image_height: 1080
camera_name: default_cam
camera_matrix:
  rows: 3
  cols: 3
  data: [821, 0, 960, 0, 821, 540, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.15, 0.02, 0, 0, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [821, 0, 960, 0, 0, 821, 540, 0, 0, 0, 1, 0]
CALEOF
        echo -e "  ${GREEN}Created!${NC}"
    else
        echo -e "  Calibration file: ${GREEN}exists${NC}"
    fi
}

ensure_packages() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}ROS 2 not found! Are you in the Dev Container?${NC}"
        exit 1
    fi

    if ! dpkg -l ros-humble-usb-cam &>/dev/null; then
        echo -e "${YELLOW}Installing required packages...${NC}"
        sudo apt update -qq && sudo apt install -y -qq ros-humble-usb-cam ros-humble-apriltag-ros v4l-utils
        echo -e "  ${GREEN}Done!${NC}"
    else
        echo -e "  ROS packages: ${GREEN}installed${NC}"
    fi
}

ensure_workspace() {
    if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
        echo -e "${YELLOW}Building workspace...${NC}"
        cd ${WORKSPACE}
        colcon build --packages-select lab9
        source install/setup.bash
        echo -e "  ${GREEN}Done!${NC}"
    else
        echo -e "  Workspace: ${GREEN}built${NC}"
    fi
}

get_env_preamble() {
    echo "export ROS_DOMAIN_ID=${ROS_DOMAIN}; \
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>${CYCLONE_IFACE}</NetworkInterfaceAddress></General></Domain></CycloneDDS>'; \
source /opt/ros/humble/setup.bash"
}

prewarm_camera() {
    echo -e "${CYAN}[0/6] Pre-warming camera (flushing bad startup frames)...${NC}"
    v4l2-ctl -d ${VIDEO_DEVICE} --stream-mmap --stream-count=30 \
        --set-fmt-video=width=${IMAGE_WIDTH},height=${IMAGE_HEIGHT},pixelformat=MJPG 2>/dev/null
    echo -e "  ${GREEN}Done — ${VIDEO_DEVICE} flushed${NC}"
    sleep 1
}

launch_camera() {
    local ENV_PREAMBLE=$(get_env_preamble)
    echo -e "${CYAN}[1/6] Launching Camera...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p video_device:=${VIDEO_DEVICE} \
        -p image_width:=${IMAGE_WIDTH} -p image_height:=${IMAGE_HEIGHT} \
        -p pixel_format:=${PIXEL_FORMAT} -p framerate:=${FRAMERATE} \
        -p frame_id:=camera_link -p camera_name:=${CAMERA_NAME} \
        -p camera_info_url:=${CAMERA_CALIB}" &
    PIDS+=($!)

    for i in $(seq ${CAMERA_WARMUP} -1 1); do
        echo -ne "  Stabilizing: ${i}s  \r"
        sleep 1
    done
    echo -e "  ${GREEN}Camera started${NC}              "
}

tune_exposure() {
    echo -e "${CYAN}[2/6] Tuning Exposure...${NC}"
    v4l2-ctl -d ${VIDEO_DEVICE} --set-ctrl=auto_exposure=1 2>/dev/null
    sleep 0.5
    v4l2-ctl -d ${VIDEO_DEVICE} --set-ctrl=exposure_time_absolute=${EXPOSURE} 2>/dev/null
    sleep 0.5
    v4l2-ctl -d ${VIDEO_DEVICE} --set-ctrl=gain=${GAIN} 2>/dev/null
    sleep 0.5
    v4l2-ctl -d ${VIDEO_DEVICE} --set-ctrl=brightness=${BRIGHTNESS} 2>/dev/null
    echo -e "  ${GREEN}Done!${NC}"
    sleep ${POST_TUNE_WAIT}
}

launch_apriltag() {
    local ENV_PREAMBLE=$(get_env_preamble)
    echo -e "${CYAN}[3/6] Launching AprilTag Detector...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        ros2 run apriltag_ros apriltag_node --ros-args \
        -r image_rect:=/image_raw -r camera_info:=/camera_info \
        -p image_transport:=raw \
        -p family:=${TAG_FAMILY} -p size:=${TAG_SIZE} -p max_hamming:=0 \
        -p z_up:=true \
        -p detector.decimate:=${DECIMATE} \
        -p detector.threads:=${THREADS} \
        -p detector.refine:=true" &
    PIDS+=($!)
    echo -e "  ${GREEN}Started${NC}"
    sleep ${POST_APRILTAG_WAIT}
}

launch_static_tf() {
    local ENV_PREAMBLE=$(get_env_preamble)
    echo -e "${CYAN}[4/6] Launching Static Transform (arena frame)...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        ros2 run tf2_ros static_transform_publisher \
        0 0 ${ARENA_HEIGHT} 1 0 0 0 arena camera_link" &
    PIDS+=($!)
    echo -e "  ${GREEN}Started${NC}"
    sleep ${POST_TF_WAIT}
}

launch_god_node() {
    local ENV_PREAMBLE=$(get_env_preamble)
    echo -e "${CYAN}[5/6] Launching God Node...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        cd ${WORKSPACE} && source install/setup.bash && \
        ros2 run lab9 god_node" &
    PIDS+=($!)
    echo -e "  ${GREEN}Started${NC}"
    sleep ${POST_GOD_WAIT}
}

launch_tractor() {
    local ENV_PREAMBLE=$(get_env_preamble)
    echo -e "${CYAN}[6/6] Launching Tractor Skeleton...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        cd ${WORKSPACE} && source install/setup.bash && \
        ros2 run lab9 tractor_skeleton" &
    PIDS+=($!)
    echo -e "  ${GREEN}Started${NC}"
}

kill_all() {
    echo -e "\n${RED}Shutting down all nodes...${NC}"
    for pid in "${PIDS[@]}"; do
        kill $pid 2>/dev/null
    done
    sleep 1
    # Force kill anything still hanging around
    for pid in "${PIDS[@]}"; do
        kill -9 $pid 2>/dev/null
    done
    pkill -f "usb_cam_node" 2>/dev/null
    pkill -f "apriltag_node" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    pkill -f "god_node" 2>/dev/null
    pkill -f "tractor_skeleton" 2>/dev/null
    wait 2>/dev/null
    echo -e "${GREEN}All stopped.${NC}"
    exit 0
}

# =====================================================================
# PIPELINE RUNNERS
# =====================================================================

run_full_pipeline() {
    echo ""
    echo -e "${BOLD}Launching Full Pipeline...${NC}"
    ensure_calibration
    ensure_packages
    ensure_workspace
    echo ""

    if ! detect_camera; then exit 1; fi
    echo ""

    trap kill_all SIGINT SIGTERM

    prewarm_camera
    launch_camera
    tune_exposure
    launch_apriltag
    launch_static_tf
    launch_god_node
    launch_tractor

    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║              ALL NODES RUNNING                          ║${NC}"
    echo -e "${GREEN}║  Press Ctrl+C to stop everything                        ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "  Domain ID:     ${CYAN}${ROS_DOMAIN}${NC}"
    echo -e "  Camera:        ${CYAN}${VIDEO_DEVICE} @ ${FRAMERATE}fps${NC}"
    echo -e "  Monitor GPS:   ${CYAN}ros2 topic echo /tractor/gps --no-daemon${NC}"
    echo -e "  Monitor vel:   ${CYAN}ros2 topic echo /cmd_vel --no-daemon${NC}"
    echo -e "  Check TF rate: ${CYAN}ros2 topic hz /tf --no-daemon${NC}"
    echo ""

    wait
}

run_camera_only() {
    echo ""
    echo -e "${BOLD}Launching Camera Pipeline (no tractor)...${NC}"
    ensure_calibration
    ensure_packages
    ensure_workspace
    echo ""

    if ! detect_camera; then exit 1; fi
    echo ""

    trap kill_all SIGINT SIGTERM

    prewarm_camera
    launch_camera
    tune_exposure
    launch_apriltag
    launch_static_tf
    launch_god_node

    echo ""
    echo -e "${GREEN}Camera pipeline running. Ctrl+C to stop.${NC}"
    echo -e "  Domain ID:   ${CYAN}${ROS_DOMAIN}${NC}"
    echo -e "  Monitor GPS: ${CYAN}ros2 topic echo /tractor/gps --no-daemon${NC}"
    echo ""

    wait
}

run_simulation() {
    local ENV_PREAMBLE=$(get_env_preamble)
    echo ""
    echo -e "${BOLD}Launching Simulation (no camera)...${NC}"
    ensure_workspace
    echo ""

    trap kill_all SIGINT SIGTERM

    echo -e "${CYAN}[1/2] Launching Interactive Arena...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        cd ${WORKSPACE} && source install/setup.bash && \
        ros2 run lab9 interactive_arena" &
    PIDS+=($!)
    sleep 3

    echo -e "${CYAN}[2/2] Launching Tractor Arena...${NC}"
    bash -c "${ENV_PREAMBLE}; \
        cd ${WORKSPACE} && source install/setup.bash && \
        ros2 run lab9 tractor_arena" &
    PIDS+=($!)

    echo ""
    echo -e "${GREEN}Simulation running. Click in the arena window to set targets.${NC}"
    echo -e "${GREEN}Ctrl+C to stop.${NC}"
    echo ""

    wait
}

# =====================================================================
# SETTINGS MENU
# =====================================================================

change_settings() {
    echo ""
    echo -e "${BOLD}Settings:${NC}"
    echo -e "  ${GREEN}1)${NC} Video Device    [${VIDEO_DEVICE:-auto}]"
    echo -e "  ${GREEN}2)${NC} Domain ID       [${ROS_DOMAIN}]"
    echo -e "  ${GREEN}3)${NC} Network Iface   [${CYCLONE_IFACE}]"
    echo -e "  ${GREEN}4)${NC} Arena Height    [${ARENA_HEIGHT}m]"
    echo -e "  ${GREEN}5)${NC} Exposure        [${EXPOSURE}]"
    echo -e "  ${GREEN}6)${NC} Gain            [${GAIN}]"
    echo -e "  ${GREEN}7)${NC} Tag Size        [${TAG_SIZE}m]"
    echo -e "  ${GREEN}8)${NC} Framerate       [${FRAMERATE}]"
    echo -e "  ${GREEN}9)${NC} Camera Warmup   [${CAMERA_WARMUP}s]"
    echo -e "  ${GREEN}b)${NC} Back to menu"
    echo ""
    read -p "Change which? " setting

    case $setting in
        1) read -p "Video device [${VIDEO_DEVICE:-auto}]: " val; VIDEO_DEVICE=${val:-$VIDEO_DEVICE} ;;
        2) read -p "Domain ID [${ROS_DOMAIN}]: " val; ROS_DOMAIN=${val:-$ROS_DOMAIN} ;;
        3) read -p "Network interface [${CYCLONE_IFACE}]: " val; CYCLONE_IFACE=${val:-$CYCLONE_IFACE} ;;
        4) read -p "Arena height [${ARENA_HEIGHT}]: " val; ARENA_HEIGHT=${val:-$ARENA_HEIGHT} ;;
        5) read -p "Exposure [${EXPOSURE}]: " val; EXPOSURE=${val:-$EXPOSURE} ;;
        6) read -p "Gain [${GAIN}]: " val; GAIN=${val:-$GAIN} ;;
        7) read -p "Tag size [${TAG_SIZE}]: " val; TAG_SIZE=${val:-$TAG_SIZE} ;;
        8) read -p "Framerate [${FRAMERATE}]: " val; FRAMERATE=${val:-$FRAMERATE} ;;
        9) read -p "Camera warmup seconds [${CAMERA_WARMUP}]: " val; CAMERA_WARMUP=${val:-$CAMERA_WARMUP} ;;
        b) return ;;
        *) echo "Invalid" ;;
    esac

    change_settings
}

menu() {
    print_banner
    print_config

    echo -e "${BOLD}What do you want to launch?${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} Full Pipeline (camera + apriltag + god node + tractor)"
    echo -e "  ${GREEN}2)${NC} Camera Only (god node publishes, no tractor control)"
    echo -e "  ${GREEN}3)${NC} Simulation Only (interactive arena + tractor, no camera)"
    echo -e "  ${GREEN}4)${NC} Change Settings"
    echo -e "  ${GREEN}q)${NC} Quit"
    echo ""
    read -p "Choose [1-4, q]: " choice

    case $choice in
        1) run_full_pipeline ;;
        2) run_camera_only ;;
        3) run_simulation ;;
        4) change_settings; menu ;;
        q) exit 0 ;;
        *) echo "Invalid choice"; menu ;;
    esac
}

# =====================================================================
# MAIN
# =====================================================================
if [ -n "$1" ]; then
    case $1 in
        1) run_full_pipeline ;;
        2) run_camera_only ;;
        3) run_simulation ;;
        *) echo "Usage: bash god_node_launcher.sh [1|2|3]"; exit 1 ;;
    esac
else
    menu
fi