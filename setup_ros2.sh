#!/bin/bash
# setup_complete.sh
# 배달 로봇 ROS2 Jazzy + YDLidar X4 Pro 완전 자동 설치

set -e  # 에러 시 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}배달 로봇 ROS2 + YDLidar 완전 자동 설치${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 시작 시간
START_TIME=$(date +%s)

# ============================================================
# 0. 시스템 확인
# ============================================================

echo -e "${YELLOW}[0/8] 시스템 확인 중...${NC}"

# Ubuntu 버전 확인
OS_VERSION=$(lsb_release -rs)
if [[ "$OS_VERSION" != "24.04" ]]; then
    echo -e "${RED}⚠️  경고: Ubuntu 24.04가 아닙니다 (현재: $OS_VERSION)${NC}"
    echo "계속하려면 Enter, 중단하려면 Ctrl+C"
    read
fi

# 아키텍처 확인
ARCH=$(uname -m)
echo -e "  아키텍처: ${GREEN}$ARCH${NC}"

# 메모리 확인
TOTAL_MEM=$(free -h | grep Mem | awk '{print $2}')
echo -e "  메모리: ${GREEN}$TOTAL_MEM${NC}"

# 디스크 공간 확인
FREE_SPACE=$(df -h / | tail -1 | awk '{print $4}')
echo -e "  여유 공간: ${GREEN}$FREE_SPACE${NC}"
echo ""

# ============================================================
# 1. 시스템 업데이트
# ============================================================

echo -e "${YELLOW}[1/8] 시스템 업데이트 중...${NC}"
echo "  (약 3-5분 소요)"

sudo apt update
sudo apt upgrade -y

echo -e "${GREEN}  ✓ 시스템 업데이트 완료${NC}"
echo ""

# ============================================================
# 2. 필수 패키지 설치
# ============================================================

echo -e "${YELLOW}[2/8] 필수 패키지 설치 중...${NC}"

sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

echo -e "${GREEN}  ✓ 필수 패키지 설치 완료${NC}"
echo ""

# ============================================================
# 3. ROS2 Jazzy 설치
# ============================================================

echo -e "${YELLOW}[3/8] ROS2 Jazzy 저장소 추가 중...${NC}"

# ROS2 GPG 키 추가
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "${GREEN}  ✓ 저장소 추가 완료${NC}"

echo -e "${YELLOW}    ROS2 Jazzy 설치 중...${NC}"
echo "  (약 10-15분 소요, 라즈베리파이는 더 오래 걸릴 수 있습니다)"

sudo apt update
sudo apt install -y ros-jazzy-desktop

echo -e "${GREEN}  ✓ ROS2 Jazzy 설치 완료${NC}"
echo ""

# ============================================================
# 4. 환경 설정
# ============================================================

echo -e "${YELLOW}[4/8] 환경 설정 중...${NC}"

# .bashrc에 자동 소싱 추가
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Jazzy" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}  ✓ .bashrc에 ROS2 소싱 추가${NC}"
else
    echo -e "${CYAN}  - ROS2 소싱 이미 존재${NC}"
fi

# 현재 세션에서 소싱
source /opt/ros/jazzy/setup.bash

# rosdep 초기화
if [ ! -d "/etc/ros/rosdep" ]; then
    sudo rosdep init
    echo -e "${GREEN}  ✓ rosdep 초기화${NC}"
else
    echo -e "${CYAN}  - rosdep 이미 초기화됨${NC}"
fi

rosdep update
echo -e "${GREEN}  ✓ rosdep 업데이트 완료${NC}"
echo ""

# ============================================================
# 5. 워크스페이스 생성
# ============================================================

echo -e "${YELLOW}[5/8] ROS2 워크스페이스 생성 중...${NC}"

# 워크스페이스 디렉토리
WS_DIR="$HOME/ros2_ws"

if [ -d "$WS_DIR" ]; then
    echo -e "${CYAN}  - 워크스페이스 이미 존재: $WS_DIR${NC}"
    read -p "  삭제하고 새로 만들까요? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$WS_DIR"
        echo -e "${YELLOW}  기존 워크스페이스 삭제${NC}"
    fi
fi

if [ ! -d "$WS_DIR" ]; then
    mkdir -p "$WS_DIR/src"
    echo -e "${GREEN}  ✓ 워크스페이스 생성: $WS_DIR${NC}"
fi

# .bashrc에 워크스페이스 소싱 추가
if ! grep -q "source $WS_DIR/install/setup.bash" ~/.bashrc; then
    echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}  ✓ .bashrc에 워크스페이스 소싱 추가${NC}"
fi

echo ""

# ============================================================
# 6. YDLidar SDK 및 ROS2 드라이버 설치
# ============================================================

echo -e "${YELLOW}[6/8] YDLidar SDK 및 ROS2 드라이버 설치 중...${NC}"
echo "  (약 5-10분 소요)"

cd "$WS_DIR/src"

# YDLidar SDK 클론
if [ ! -d "YDLidar-SDK" ]; then
    echo -e "${CYAN}  YDLidar SDK 클론 중...${NC}"
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
else
    echo -e "${CYAN}  - YDLidar SDK 이미 존재${NC}"
    cd YDLidar-SDK
    git pull
    cd ..
fi

# YDLidar SDK 빌드 및 설치
cd YDLidar-SDK
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install

echo -e "${GREEN}  ✓ YDLidar SDK 설치 완료${NC}"

cd "$WS_DIR/src"

# YDLidar ROS2 드라이버 클론
if [ ! -d "ydlidar_ros2_driver" ]; then
    echo -e "${CYAN}  YDLidar ROS2 드라이버 클론 중...${NC}"
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
else
    echo -e "${CYAN}  - YDLidar ROS2 드라이버 이미 존재${NC}"
    cd ydlidar_ros2_driver
    git pull
    cd ..
fi

echo -e "${GREEN}  ✓ YDLidar ROS2 드라이버 클론 완료${NC}"
echo ""

# ============================================================
# 7. 워크스페이스 빌드
# ============================================================

echo -e "${YELLOW}[7/8] 워크스페이스 빌드 중...${NC}"
echo "  (약 5-10분 소요, 라즈베리파이는 더 오래 걸립니다)"

cd "$WS_DIR"

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install

# 소싱
source "$WS_DIR/install/setup.bash"

echo -e "${GREEN}  ✓ 워크스페이스 빌드 완료${NC}"
echo ""

# ============================================================
# 8. USB 권한 및 udev 규칙 설정
# ============================================================

echo -e "${YELLOW}[8/8] USB 권한 및 udev 규칙 설정 중...${NC}"

# dialout 그룹 추가
if ! groups | grep -q dialout; then
    sudo usermod -a -G dialout $USER
    echo -e "${GREEN}  ✓ dialout 그룹 추가 (재부팅 필요)${NC}"
else
    echo -e "${CYAN}  - dialout 그룹 이미 포함${NC}"
fi

# udev 규칙 생성
UDEV_FILE="/etc/udev/rules.d/99-ydlidar.rules"

if [ ! -f "$UDEV_FILE" ]; then
    echo -e "${CYAN}  udev 규칙 생성 중...${NC}"
    
    sudo bash -c "cat > $UDEV_FILE" << 'EOF'
# YDLidar X4 Pro
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"

# YDLidar 일반 규칙
KERNEL=="ttyUSB*", ATTRS{manufacturer}=="Silicon Labs", MODE:="0666", GROUP:="dialout"
EOF

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    echo -e "${GREEN}  ✓ udev 규칙 생성 완료${NC}"
else
    echo -e "${CYAN}  - udev 규칙 이미 존재${NC}"
fi

echo ""

# ============================================================
# 완료 및 안내
# ============================================================

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))
MINUTES=$((ELAPSED / 60))
SECONDS=$((ELAPSED % 60))

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✅ 설치 완료!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "소요 시간: ${CYAN}${MINUTES}분 ${SECONDS}초${NC}"
echo ""

# 설치된 버전 확인
echo -e "${YELLOW}📋 설치 정보:${NC}"
echo -e "  ROS2 버전: ${GREEN}$(ros2 --version 2>/dev/null || echo 'N/A')${NC}"
echo -e "  Python: ${GREEN}$(python3 --version)${NC}"
echo -e "  Workspace: ${GREEN}$WS_DIR${NC}"
echo ""

# 다음 단계
echo -e "${YELLOW}🎯 다음 단계:${NC}"
echo ""
echo -e "1️⃣  ${CYAN}재부팅 (권한 적용)${NC}"
echo "   sudo reboot"
echo ""
echo -e "2️⃣  ${CYAN}LiDAR 연결 확인 (재부팅 후)${NC}"
echo "   ls -l /dev/ydlidar /dev/ttyUSB*"
echo ""
echo -e "3️⃣  ${CYAN}LiDAR 테스트 실행${NC}"
echo "   source ~/.bashrc"
echo "   ros2 launch ydlidar_ros2_driver ydlidar_launch.py"
echo ""
echo -e "4️⃣  ${CYAN}데이터 확인 (다른 터미널)${NC}"
echo "   ros2 topic list"
echo "   ros2 topic echo /scan"
echo ""
echo -e "5️⃣  ${CYAN}시각화 (RViz2)${NC}"
echo "   rviz2"
echo ""

# 주의사항
echo -e "${RED}⚠️  중요:${NC}"
echo "  - 반드시 재부팅해야 USB 권한이 적용됩니다!"
echo "  - LiDAR를 USB에 연결한 후 테스트하세요."
echo ""

# 문제 해결
echo -e "${YELLOW}🔧 문제 해결:${NC}"
echo "  - 권한 오류: groups 명령어로 dialout 확인"
echo "  - 장치 없음: ls -l /dev/ttyUSB* 확인"
echo "  - 빌드 오류: cd ~/ros2_ws && colcon build --symlink-install"
echo ""

# 문서 위치
echo -e "${YELLOW}📚 추가 문서:${NC}"
echo "  - 빠른 참조: QUICK_REFERENCE.md"
echo "  - X4 설정: ydlidar_x4_launch.py"
echo "  - 테스트: test_ydlidar.py"
echo ""

echo -e "${GREEN}Happy Coding! 🤖🚚${NC}"
echo ""
