FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

RUN apt-get update && apt install -y \
    # Qt6
    qt6-base-dev qt6-tools-dev libgl1-mesa-dev \
    # clangd 
    wget curl software-properties-common gnupg

# ROS2 humble
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt update && apt install -y curl gnupg2 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu \
        $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt update && apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# setting clangd 21 version(https://apt.llvm.org/)
RUN wget https://apt.llvm.org/llvm.sh -O /tmp/llvm.sh \
    && chmod +x /tmp/llvm.sh \ 
    && /tmp/llvm.sh 21 \
    && ln -s /usr/bin/clangd-21 /usr/bin/clangd 

# cuda toolkit
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin \
&& mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
&& wget https://developer.download.nvidia.com/compute/cuda/13.0.2/local_installers/cuda-repo-ubuntu2204-13-0-local_13.0.2-580.95.05-1_amd64.deb \
&& dpkg -i cuda-repo-ubuntu2204-13-0-local_13.0.2-580.95.05-1_amd64.deb \
&& cp /var/cuda-repo-ubuntu2204-13-0-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
&& apt-get update \
&& apt-get -y install cuda-toolkit-13-0

# Using non-root system
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USERNAME -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
    
USER $USERNAME
