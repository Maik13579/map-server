ARG ROS_DISTRO=humble

FROM ros-base-image:$ROS_DISTRO

# Install deps
RUN apt update \
 && apt install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
 && pip3 install scipy \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws

# Get source code
COPY python /root/python
COPY ros2 src/ros2 

# Build python package
RUN cd /root/python && python3 setup.py bdist_wheel sdist && pip3 install -e .

# Build ros package
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

COPY example /example