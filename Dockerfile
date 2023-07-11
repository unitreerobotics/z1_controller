FROM  kstein/noetic-arm-desktop-full

RUN apt-get update
RUN apt-get -y install python3-catkin-tools ninja-build git telnet

ADD . / data/
WORKDIR data
RUN mkdir build
WORKDIR build
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic
RUN bash -c 'cmake ..'
RUN make
WORKDIR ..
RUN ./prepare_unitree_ws.sh
SHELL ["/bin/bash", "-c"]
WORKDIR unitree_ws
RUN source /ros_entrypoint.sh && /opt/ros/noetic/bin/catkin_make
RUN source devel/setup.bash
# WORKDIR ../build
#CMD roslaunch unitree_gazebo z1_spawn.launch

#CMD ./z1_ctrl
#docker tag bytelogics:z1_controller siilats/z1_controller
#docker push siilats/z1_controller:latest
#xhost +localhost
#defaults read org.xquartz.X11 enable_iglx

#defaults write org.xquartz.X11 enable_iglx -bool YES
#defaults read org.xquartz.X11 enable_iglx


#docker run  -ti -v /tmp/.X11-unix:/tmp/.X11-unix -e "DISPLAY=host.docker.internal:0" pstoll/emacs-cross emacs
# docker run -it -p 11311:11311 --env="ROS_MASTER_URI=http://localhost:11311" bytelogics:z1_controller roslaunch unitree_gazebo z1_spawn.launch

#docker run -p 6080:80 --shm-size=512m tiryoh/ros-desktop-vnc:melodic