ARG ROS_DISTRO=noetic

FROM ros-base-image:$ROS_DISTRO

## Install deps
#RUN apt update \
# && apt install -y \
#    stuff_to_install \
# && rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws

# Get source code
COPY python /root/python
COPY ros src/ros

# Build python package
RUN cd /root/python && python3 setup.py bdist_wheel sdist && pip3 install -e .

# Build ros package
RUN source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make

COPY example /example
