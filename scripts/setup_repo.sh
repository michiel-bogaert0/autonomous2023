#! /bin/bash

# Make sure to run this script from the autonomous directory using scripts/setup_repo.sh

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
    libyaml-cpp-dev \
    libcurl4-openssl-dev

# Install AirSim deps
AirSim/setup.sh

# Add FSDS settings
mkdir ~/Formula-Student-Driverless-Simulator
cp settings.json ~/Formula-Student-Driverless-Simulator/

# Setup some handy bashrc lines
echo 'export PATH="'"/home/$(id -un)/.local/bin"':$PATH''"' >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/autonomous2023/ROS/devel/setup.bash" >> ~/.bashrc
echo "alias sdev=\"source devel/setup.bash\"" >> ~/.bashrc