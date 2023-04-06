#
# Base stage. Install repository dependencies
#
FROM osrf/ros:noetic-desktop-full AS ugr-base

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
        git

RUN export TZ=Europe/Brussels

# Add ugr user
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

# Install dependencies from our repo. Uses a different folder to prevent conflicts with mounting
COPY /scripts scripts
COPY /AirSim AirSim
COPY /settings.json settings.json
COPY requirements.txt requirements.txt
COPY requirements_pc.txt requirements_pc.txt

RUN bash scripts/docker/init.sh

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

EXPOSE 11311
EXPOSE 9090
EXPOSE 7502/udp
EXPOSE 7503/udp

CMD /bin/zsh
