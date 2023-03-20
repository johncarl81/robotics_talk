FROM ros:humble-ros-base

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install Dependencies
RUN apt-get -qq install ros-humble-desktop ros-humble-mavros rsync wget git python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml gazebo libgazebo-dev ros-humble-gazebo-ros-pkgs python-is-python3

RUN mkdir -p $WORKSPACE/gazebo

# Install MavProxy
RUN pip install MAVProxy pexpect

RUN git clone https://github.com/osrf/gazebo_models $WORKSPACE/models

# Setup Geographic dataset
RUN mkdir -p $WORKSPACE/data && \
    cd $WORKSPACE/data && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod 700 ./install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh

# Install simulation
RUN cd $WORKSPACE && \
    git clone --recurse-submodules -j8 --depth 1 https://github.com/ArduPilot/ardupilot.git

RUN cd $WORKSPACE && \
    git clone --recurse-submodules -j8 --depth 1 https://github.com/SwiftGust/ardupilot_gazebo.git

RUN cd $WORKSPACE/ardupilot && \
    ./waf configure && ./waf copter

RUN cd $WORKSPACE/ardupilot_gazebo && \
    mkdir build && cd build && cmake .. && make -j4 && make install

COPY ./gazebo $WORKSPACE/gazebo

RUN source /opt/ros/humble/setup.bash && \
    cd $WORKSPACE/gazebo && \
    colcon build

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
COPY ./docker/entrypoint.py /

ENTRYPOINT ["/entrypoint.py"]
