FROM ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LIVOX_SDK2_DIR=/usr/local

RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    dbus-x11 \
    git \
    iproute2 \
    libapr1-dev \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libomp-dev \
    libopencv-dev \
    libpcap-dev \
    libpcl-dev \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    locales \
    mesa-utils \
    net-tools \
    python3-colcon-common-extensions \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    qt5-qmake \
    qtbase5-dev \
    sudo \
    tigervnc-common \
    tigervnc-standalone-server \
    wget \
    x11-apps \
    x11-xserver-utils \
    xfce4-panel \
    xfce4-session \
    xfce4-terminal \
    xfdesktop4 \
    xfwm4 \
    ros-humble-laser-geometry \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-transport \
    ros-humble-rqt-common-plugins \
    ros-humble-sensor-msgs-py \
    ros-humble-tf-transformations \
    ros-humble-tf2-sensor-msgs \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/Livox-SDK2/build --parallel "$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && rm -rf /tmp/Livox-SDK2

RUN useradd -m -s /bin/bash zerohour \
    && echo "zerohour ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/zerohour \
    && chmod 0440 /etc/sudoers.d/zerohour

WORKDIR /home/zerohour/xjtu_nav26
COPY --chown=zerohour:zerohour . .
RUN chmod +x docker/entrypoint.sh script/*.sh

USER zerohour
RUN source /opt/ros/humble/setup.bash && ./build.sh

ENTRYPOINT ["/home/zerohour/xjtu_nav26/docker/entrypoint.sh"]
CMD ["bash"]
