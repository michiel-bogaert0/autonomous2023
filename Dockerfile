#
# Base stage. Install repository dependencies
#
FROM nvidia/cuda:11.7.0-runtime-ubuntu20.04 AS ugr-base

#
# Install torch and torchvision
#
RUN apt-get update && apt-get -y install --no-install-recommends python3 python3-pip
RUN pip install torch==2.0.0
RUN pip install torchvision==0.15.1

#
# Initial dependencies
#

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    sudo \
    curl \
    lsb-release \
    gnupg2 \
    apt-utils \
    wget \
    tzdata \
    keyboard-configuration \
    git \ 
    wget \
    rsync \
    unzip \
    g++ \
    vim \
    curl \
    lsb-release \
    mesa-utils \
    htop \
    python3-opencv

RUN export TZ=Europe/Brussels

#
# Install ROS
#

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install -y ros-noetic-desktop
RUN bash /opt/ros/noetic/setup.bash
RUN sudo apt-get install -y python3-rosdep
RUN sudo rosdep init
RUN sudo rosdep update

# ROS deps
RUN apt-get update && apt-get install -y ros-noetic-tf2-geometry-msgs \
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
    ros-noetic-rosbridge-suite \
    ros-noetic-catkin-virtualenv \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rosparam-shortcuts \
    python3-catkin-tools \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    python3-pcl \
    pcl-tools

#
# Other dependencies
#

# UGR user
RUN useradd ugr -s /bin/zsh && echo "ugr:ugr" | chpasswd && adduser ugr sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ugr

WORKDIR /home/ugr
RUN mkdir autonomous2023

# Install ZSH
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.3/zsh-in-docker.sh)" -- \
    -t robbyrussell \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions


# Python stuff
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

RUN wget -O neoapi-1.2.1-cp38-cp38-linux_x86_64.whl https://github.com/UgentRacing/autonomous_public_binaries/blob/main/modules/neoapi-1.2.1-cp38-cp38-linux_x86_64.whl?raw=true
RUN python3 -m pip install neoapi-1.2.1-cp38-cp38-linux_x86_64.whl

# Handy commands
RUN echo 'export PATH="'"/home/$(id -un)/.local/bin"':$PATH''"' >> ~/.zshrc && \
    echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc && \
    echo "source ~/autonomous2023/ROS/devel/setup.zsh" >> ~/.zshrc && \
    echo "alias sdev=\"source ~/autonomous2023/ROS/devel/setup.zsh\"" >> ~/.zshrc && \
    echo "alias ugr=\"cd ~/autonomous2023/ROS/\"" >> ~/.zshrc && \
    echo "alias cbuild='catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.zshrc && \ 
    echo "source /home/ugr/autonomous2023/env-vars.sh" >> ~/.zshrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

CMD /bin/zsh
