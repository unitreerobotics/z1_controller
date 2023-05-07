mkdir unitree_ws
cd unitree_ws
mkdir src
mkdir build
cd src
git clone git@github.com:Bytelogics/unitree_ros.git
git clone git@github.com:Bytelogics/unitree_ros_to_real.git
rm -rf unitree_ros_to_real/unitree_legged_real