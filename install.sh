sudo apt-get install libeigen-dev
cd ~/chaser_ws/src

# qpOASES
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake .. -DCMAKE_CXX_FLAGS=-fPIC
sudo make install

# trajgen
cd ~/chaser_ws/src
git clone https://github.com/icsl-Jeon/traj_gen.git
cd ./traj_gen/cpp
mkdir build && cd build
cmake ..
make && sudo make install

# octomap stuff
cd ~/chaser_ws/src
sudo apt-get install ros-noetic-octomap-ros
git clone https://github.com/icsl-Jeon/octomap
cd octomap/dynamicEDT3D
mkdir build && cmake ..
sudo make install

# catkin pkg all build
sudo apt-get install ros-${ROS_DISTRO}-compressed-depth-image-transport
cd ~/chaser_ws/src
git clone https://github.com/icsl-Jeon/dual_chaser_msgs
git clone https://github.com/icsl-Jeon/octomap_mapping
git clone /github.com/icsl-Jeon/chasing_utils.git

# zed2_client
git clone https://github.com/stereolabs/zed-ros-wrapper.git
git clone https://github.com/icsl-Jeon/zed-ros-examples.git
git clone https://github.com/icsl-Jeon/zed2_client.git
git clone https://github.com/stereolabs/zed-ros-interfaces
cd ~/chaser_ws
ls src
catkin build zed_interfaces rviz_plugin_zed_od zed2_client dual_chaser_msgs octomap_server chasing_utils dual_chaser
