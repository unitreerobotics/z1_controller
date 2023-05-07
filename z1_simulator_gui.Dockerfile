FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get -y install python3-catkin-tools ros-noetic-catkin

ADD unitree_ws /data/unitree_ws
SHELL ["/bin/bash", "-c"]
WORKDIR data/unitree_ws/
RUN source /ros_entrypoint.sh && /opt/ros/noetic/bin/catkin_make
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic

CMD source devel/setup.bash && roslaunch unitree_gazebo z1.launch
