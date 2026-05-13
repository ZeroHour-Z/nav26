FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LIVOX_SDK2_DIR=/usr/local
ENV ROS_DISTRO=humble

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
    libssl-dev \
    libsuitesparse-dev \
    libusb-1.0-0-dev \
    libyaml-cpp-dev \
    locales \
    mesa-utils \
    net-tools \
    pkg-config \
    python3-yaml \
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
    ros-humble-ament-cmake-auto \
    ros-humble-ament-cmake-python \
    ros-humble-action-msgs \
    ros-humble-common-interfaces \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-laser-geometry \
    ros-humble-message-filters \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-nav2-rviz-plugins \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-py-trees \
    ros-humble-py-trees-js \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-rclcpp-components \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-transport \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-sensor-msgs-py \
    ros-humble-std-srvs \
    ros-humble-tf-transformations \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-vision-opencv \
    ros-humble-visualization-msgs \
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
RUN chmod +x docker/entrypoint.sh build.sh nav.sh set_param.sh script/*.sh

RUN if [ ! -f src/rm_driver/livox_ros_driver2/package.xml ]; then \
        cp src/rm_driver/livox_ros_driver2/package_ROS2.xml src/rm_driver/livox_ros_driver2/package.xml; \
    fi \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO" \
        --skip-keys "python3-open3d opencv livox_ros2_driver" \
    && rm -rf /var/lib/apt/lists/* \
    && chown -R zerohour:zerohour /home/zerohour/xjtu_nav26

USER zerohour
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f /home/zerohour/xjtu_nav26/install/setup.bash ] && source /home/zerohour/xjtu_nav26/install/setup.bash" >> ~/.bashrc \
    && source /opt/ros/humble/setup.bash \
    && ./build.sh

ENTRYPOINT ["/home/zerohour/xjtu_nav26/docker/entrypoint.sh"]
CMD ["bash"]
