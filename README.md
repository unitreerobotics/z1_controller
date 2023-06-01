documentation

[unitree-z1-docs-english](http://dev-z1.unitree.com)

[unitree-z1-docs-chinese](http://dev-z1.cn.unitree.com)

see [unitree-z1-docs](http://dev-z1.unitree.com)

## Build docker image
```shell
docker build . -t bytelogics:z1_controller
```

## Run the docker instance
For python connection:
```shell
docker run -ti --rm --network host bytelogics:z1_controller

```

For manual build within a docker:
Make sure you are not in build folder but in the parent then run
docker run -it --net=host --env="ROS_MASTER_URI=http://host.docker.internal:11311" bytelogics:z1_controller bash
`docker run -ti --rm --network host bytelogics:z1_controller bash`
`cmake .. -DCOMMUNICATION=ROS`
`make`
`./z1_ctrl`
you can try with 
`./z1_ctrl k` for keyboard

Cmake command you can do catkin_make -DCOMMUNICATION=ROS 
then you dont need to edit the cmake.txt file
## old docker run -ti --rm --network host --env="DISPLAY"  -e DISPLAY=${HOSTNAME}:0 --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --volume="/Users/keithsiilats/overlay_ws:/data/overlay_ws" --volume="/Users/keithsiilats/unitree_ws:/data/unitree_ws"  -p 8071:8071 -p 11311:11311 bytelogics:z1_controller bash
docker run -ti --rm --network host    --env="DISPLAY=novnc:0.0" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --volume="/Users/keithsiilats/overlay_ws:/data/overlay_ws" --volume="/Users/keithsiilats/unitree_ws:/data/unitree_ws"  -p 8071:8071 -p 11311:11311 bytelogics:z1_controller bash
docker run -d --rm --network host  --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" \
--name=novnc -p=8080:8080 theasp/novnc:latest

## on mac setup 
https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285
cd /data/unitree_ws
source devel/setup.bash
roslaunch unitree_gazebo z1.launch

## macos v2 
brew install --cask mambaforge
mamba create -n ros_env
mamba init
mamba activate ros_env
conda config --env --add channels robostack-staging
mamba install ros-noetic-desktop-full
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools
./prepare_unitree_ws.sh
cd unitree_ws
rm -Rf src/unitree_controller
rm -Rf src/z1_controller
rm -Rf src/unitree_legged_control

catkin_make
source devel/setup.bash 
roslaunch unitree_gazebo z1_empty.launch
# go to dockcker and run
# roslaunch unitree_gazebo z1_spawn.launch

