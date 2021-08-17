# dual_chaser
*chasing algorithm for motion planning in obstacle environment*
[![thumbnail](https://user-images.githubusercontent.com/30062474/129690622-0c8a87ca-fe70-4516-b26d-78de3ab56c2d.png)](https://youtu.be/RE6pJ6QvqsA)



## Installation 

### [traj_gen](https://github.com/icsl-Jeon/traj_gen)

```
git clone https://github.com/icsl-Jeon/traj_gen.git
cd ./traj_gen/cpp
mkdir build && cd build
cmake ..
make && sudo make install
```

### [dual_chaser_msgs](https://github.com/icsl-Jeon/dual_chaser_msgs)
```
cd ~/catkin_ws/src
git clone https://github.com/icsl-Jeon/dual_chaser_msgs
cd ../
catkin build
```

### [chasing_utils](https://github.com/icsl-Jeon/chasing_utils.git)
Follow the installation instruction of the repo:)

### [zed2_client (optinal) ](https://github.com/icsl-Jeon/zed2_client.git)
Follow the installation instruction of the repo:)

## Launch 

To launch the algorithm, first download [bag files](https://drive.google.com/drive/folders/1AtZIgeRLxQMqIC9SMKBOhj9OXK96uEfw?usp=sharing). 
The bags were recorded with a zed2 camera including [object detection messages](https://www.stereolabs.com/docs/ros/object-detection/).
Here, we use [zed2_client](#zed2_client-optinal-httpsgithubcomicsl-jeonzed2_clientgit) 
to provide `/tf` of targets and pointcloud `~cloud_in`.  
```
roslaunch dual_chaser zed_online.launch 
```

## Required transforms and topic 

### 1. tf 
* World frame `frame_id` to drone (or sensor frame) `drone_frame_id`. Defaults are `map` and `base_link` respectively. 
* World frame to target object frames `{target_0_filtered,target_1_filtered}` (both required). 

### 2. topics 
* Pointcloud `~cloud_in` for [EdtOctomapServer](https://github.com/icsl-Jeon/octomap_mapping/blob/kinetic-devel/octomap_server/include/octomap_server/EdtOctomapServer.h)

## Code structure

![img](img/structure.png)



## Analyzing
The [status message](https://github.com/icsl-Jeon/dual_chaser_msgs) regarding the current planning pose is published 
in the topic `~/wrapper/status`. This can be visualized in [rqt_gui](rviz/monitor.perspective) or in [matlab](rosbag/status_log.m)

