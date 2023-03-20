FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install Dependencies
RUN apt-get -qq install ros-humble-desktop python3-dev python-is-python3

RUN apt-get -qq install tree

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*
