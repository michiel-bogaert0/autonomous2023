#! /bin/zsh

# Make sure to run this script from the autonomous directory using scripts/setup_repo_jetson.zsh

# Install some useful Ubuntu packages
sudo apt update
sudo apt install -y \
    python3 \
	python3-pip \
    wget \
	rsync \
	unzip \
	g++ \
	vim \
    curl \
    lsb-release \
    mesa-utils \
    htop 
    

# Install the python requirements
pip install -r requirements.txt

# Install ROS requirements
sudo apt-get install -y ros-noetic-tf2-geometry-msgs \
    python3-catkin-tools \
    ros-noetic-rqt-multiplot  \
    ros-noetic-joy \
    ros-noetic-perception-pcl \
    ros-noetic-image-transport \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-robot-localization \
    ros-noetic-laser-assembler \
    ros-noetic-perception-pcl \
    ros-noetic-can-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-nmea-navsat-driver \
    ros-noetic-catkin-virtualenv \
    python3-catkin-tools \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    python3-pcl \
    pcl-tools

# Setup some handy zshrc lines
echo 'export PATH="'"/home/$(id -un)/.local/bin"':$PATH''"' >> ~/.zshrc
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
echo "source ~/autonomous2023/ROS/devel/setup.zsh" >> ~/.zshrc
echo "alias sdev=\"source ~/autonomous2023/ROS/devel/setup.zsh\"" >> ~/.zshrc
echo "alias ugr=\"cd ~/autonomous2023/ROS/\"" >> ~/.zshrc
echo "alias cbuild='catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.zshrc
