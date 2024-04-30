#!/bin/bash

# Reference sites
# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

echo ""
echo "[Setting...]"
echo "[Note] OS version >>> Ubuntu 20.04 (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS 2 Foxy Fitzroy"
echo ""
echo "PRESS [ENTER] to CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTTL + C]"
read

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo ""
echo "[Add ROS2 repository]"
echo ""
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo ""
echo "[Update the package]"
echo ""
sudo apt update

sudo apt upgrade

echo ""
echo "[Install ROS2 and ROS2 Pakcage]"
echo ""
sudo apt install ros-foxy-desktop python3-argcomplete

sudo apt install ros-dev-tools

echo ""
echo "[Install ROS2 Dev Tools]"
echo ""
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

sudo apt install --no-install-recommends -y \
  libcunit1-dev

echo ""
echo "[how to use]"
echo "ROS2 Build Test"
echo "\$ source /opt/ros/foxy/setup.bash (call C:\\dev\\ros2-eloquent\\setup.bat)"
echo "\$ mkdir -p ~/robot_ws/src"
echo "\$ cd robot_ws/"
echo "\$ colcon build --symlink-install"
echo ""
echo "src 이동 후 build type에 맞는 패키지 생성"
echo "ros2 pkg create --build-type ament_cmake <my_package_name>"
echo "ros2 pkg create --build-type ament_python <my_package_name>"
echo ""
echo "executable 만들기 (안쓸듯?)"
echo "ros2 pkg create --build-type ament_cmake --node-name my_node my_package"
echo "ros2 pkg create --build-type ament_python --node-name my_node my_package"
echo ""
echo "ws 이동 후 colcon build"
echo "특정 패키지만 빌드 : colcon build --packages-select my_package"
echo ""
echo "빌드 후 ws에서 source install/setup.bash (call C:\\Users\\SSAFY\\Desktop\\catkin_ws\\install\\local_setup.bat)"
echo "후 ros2 run <package명> <node명> 으로 노드 실행"


