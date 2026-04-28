# nav-frontier-go2w-v3 — single docker image for the Go2W frontier exploration stack.
#
# Builds the full ROS 2 humble workspace into the image so the container is
# launch-ready: `docker run ... ros2 launch nav_frontier_go2w_bringup bringup.launch.py`.

FROM ubuntu:jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 Humble base + dev tools.
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        software-properties-common \
        ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update -q && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        python3-argcomplete \
        tmux \
        git \
        wget \
        net-tools \
        iputils-ping && \
    rm -rf /var/lib/apt/lists/*

# Nav2 + SLAM + sensor message glue.
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-slam-toolbox \
        ros-humble-nav2-bringup \
        ros-humble-nav2-mppi-controller \
        ros-humble-nav2-navfn-planner \
        ros-humble-nav2-velocity-smoother \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-bt-navigator \
        ros-humble-nav2-simple-commander \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-pcl-conversions \
        ros-humble-pcl-ros \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-tf-transformations && \
    rm -rf /var/lib/apt/lists/*

# System libraries needed by D-LIO (Eigen, PCL, OpenMP) and the Hesai SDK (libpcap).
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libeigen3-dev \
        libomp-dev \
        libpcl-dev \
        libpcap-dev && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        transforms3d \
        websocket-client \
        catmux \
        distro

WORKDIR /workspace/humble_ws

# CycloneDDS profile available before colcon build (msg packages need it).
COPY config/cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Build the workspace at image-build time.
COPY humble_ws/src/ /workspace/humble_ws/src/
RUN source /opt/ros/humble/setup.bash && \
    rosdep init 2>/dev/null || true && \
    rosdep update --rosdistro humble 2>/dev/null || true && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/humble_ws/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=0

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
