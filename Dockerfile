FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get -y install python3-catkin-tools

ADD . / data/
WORKDIR data
RUN mkdir build
WORKDIR build
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic
RUN bash -c 'cmake ..'
RUN make

