FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        dbus-x11 \
        iproute2 \
        iputils-ping \
        less \
        mesa-utils \
        python3-colcon-common-extensions \
        ros-humble-nav2-msgs \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rviz2 \
        ros-humble-tf2-tools \
        xauth && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV QT_X11_NO_MITSHM=1

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
