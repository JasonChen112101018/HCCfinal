#!/bin/bash
# Tello無人機系統自動化安裝腳本
# 使用方法: chmod +x install_tello_system.sh && ./install_tello_system.sh

set -e  # 遇到錯誤時停止執行

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日誌函數
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 檢查是否為root用戶
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_error "請不要以root用戶運行此腳本"
        exit 1
    fi
}

# 檢查Ubuntu版本
check_ubuntu_version() {
    if [[ ! -f /etc/os-release ]]; then
        log_error "無法檢測作業系統版本"
        exit 1
    fi
    
    source /etc/os-release
    if [[ "$ID" != "ubuntu" ]]; then
        log_error "此腳本僅支援Ubuntu系統"
        exit 1
    fi
    
    case "$VERSION_ID" in
        "20.04"|"22.04")
            log_success "檢測到Ubuntu $VERSION_ID，支援的版本"
            ;;
        *)
            log_warning "檢測到Ubuntu $VERSION_ID，建議使用20.04或22.04"
            ;;
    esac
}

# 更新系統
update_system() {
    log_info "更新系統套件..."
    sudo apt update
    sudo apt upgrade -y
    log_success "系統更新完成"
}

# 安裝ROS 2 Foxy
install_ros2() {
    log_info "安裝ROS 2 Foxy..."
    
    # 檢查是否已安裝
    if command -v ros2 &> /dev/null; then
        log_success "ROS 2已安裝，跳過"
        return
    fi
    
    # 設置locale
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    # 添加ROS 2 apt repository
    sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # 安裝ROS 2
    sudo apt update
    sudo apt install -y ros-foxy-desktop python3-argcomplete
    
    # 安裝開發工具
    sudo apt install -y python3-colcon-common-extensions python3-vcstool python3-rosdep
    
    # 初始化rosdep
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # 設置環境變數
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    
    log_success "ROS 2 Foxy安裝完成"
}

# 安裝系統依賴
install_system_dependencies() {
    log_info "安裝系統依賴..."
    
    # 基本開發工具
    sudo apt install -y \
        python3-pip \
        python3-dev \
        python3-venv \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        vim \
        htop \
        tree \
        unzip
    
    # OpenCV依賴
    sudo apt install -y \
        libopencv-dev \
        python3-opencv \
        libgtk-3-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        gfortran \
        libatlas-base-dev \
        libhdf5-dev \
        pkg-config
    
    # AprilTag依賴
    sudo apt install -y \
        libeigen3-dev \
        libceres-dev
    
    log_success "系統依賴安裝完成"
}

# 安裝CUDA (可選)
install_cuda() {
    log_info "檢查NVIDIA GPU..."
    
    if ! command -v nvidia-smi &> /dev/null; then
        log_warning "未檢測到NVIDIA GPU，跳過CUDA安裝"
        return
    fi
    
    log_info "檢測到NVIDIA GPU，是否安裝CUDA? (y/N)"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        log_info "安裝CUDA..."
        
        # 檢查是否已安裝
        if command -v nvcc &> /dev/null; then
            log_success "CUDA已安裝，跳過"
            return
        fi
        
        # 下載並安裝CUDA
        cd /tmp
        wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
        sudo sh cuda_11.8.0_520.61.05_linux.run --silent --toolkit
        
        # 設置環境變數
        echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
        echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
        
        log_success "CUDA安裝完成"
    fi
}

# 創建工作空間
create_workspace() {
    log_info "創建工作空間..."
    
    WORKSPACE_DIR="$HOME/tello_workspace"
    
    if [ -d "$WORKSPACE_DIR" ]; then
        log_warning "工作空間已存在，是否刪除重建? (y/N)"
        read -r response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
            rm -rf "$WORKSPACE_DIR"
        else
            log_info "使用現有工作空間"
            cd "$WORKSPACE_DIR"
            return
        fi
    fi
    
    mkdir -p "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
    
    # 創建虛擬環境
    log_info "創建Python虛擬環境..."
    python3 -m venv tello_env
    source tello_env/bin/activate
    pip install --upgrade pip setuptools wheel
    
    # 創建ROS 2工作空間
    log_info "創建ROS 2工作空間..."
    mkdir -p tello_ros2_ws/src
    cd tello_ros2_ws
    
    # 創建packages
    source /opt/ros/foxy/setup.bash
    cd src
    ros2 pkg create --build-type ament_python tello_controller_node
    ros2 pkg create --build-type ament_python yolov5_detector_node
    ros2 pkg create --build-type ament_python apriltag_detector_node
    ros2 pkg create --build-type ament_python navigation_node
    ros2 pkg create --build-type ament_python mission_manager_node
    ros2 pkg create --build-type ament_cmake tello_msgs
    
    # 創建其他目錄
    cd "$WORKSPACE_DIR"
    mkdir -p models calibration launch logs
