# Use the official ROS Noetic + Gazebo image
FROM osrf/ros:noetic-desktop-full

# avoid prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# install tools & dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential cmake git wget curl lsb-release gnupg2 \
      libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
      libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libudev-dev \
      python3-colcon-common-extensions python3-pip \
      && rm -rf /var/lib/apt/lists/*

# build librealsense from source
RUN mkdir -p /opt/realsense_src && cd /opt/realsense_src && \
    git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && git checkout v2.53.1 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# install ROS RealSense wrapper & UR packages
RUN . /opt/ros/noetic/setup.sh && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-noetic-ur-gazebo ros-noetic-ur-description \
      ros-noetic-ur-robot-driver \
      ros-noetic-realsense2-camera ros-noetic-realsense2-description \
      ros-noetic-moveit \
      && rm -rf /var/lib/apt/lists/*

# setup a catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# (optional) clone your own ROS nodes into src here
# COPY src/ /root/catkin_ws/src/

# build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# entrypoint for easier ROS usage
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"] 