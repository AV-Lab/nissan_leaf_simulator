# Nissan leaf description

## First clone the repo
```
git clone https://github.com/AV-Lab/nissan_leaf
```
**ROS Packages**
```
sudo apt-get update
sudo apt-get install ros-noetic-controller-manager
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-navfn
sudo apt-get install ros-noetic-global-planner
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-velodyne-pointcloud
sudo apt-get install ros-noetic-pointcloud-to-laserscan
sudo apt-get install libignition-rendering3
```
**Clone the Ouster lidar repo under the same workspace**
```
git clone --recursive https://github.com/gepetto/ouster-gazebo-simulation.git
```
**Install dependencies**
```
rosdep install --from-paths src --ignore-src -r -y
```
**Build & source the workspace**
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
## Nissan leaf Simulation

**Spawn the car in an empty world**
```
roslaunch catvehicle catvehicle_empty.launch
```

**Or run nissan leaf simulation in a test world environment**
```
roslaunch catvehicle catvehicle_spawn.launch
```
The launch file will start the following: 
1) Catvehicle.launch
2) points_to_scan as the ouster lidar produces pointcloud which gets converted to scan
3) move_base.launch for the navigation stack
4) Rviz for visualization 

**Set a navigation goal on RVIZ**

![](/assets/images/Set_navigation_goal.png)

**To run gmapping in a new .world file**

1) Update world_name variable in the catvehicle_empty.launch file to the new world created 
```
<arg name="world_name" value="$(find catvehicle)/worlds/plane.world"/>
```
2) Save and run catvehicle empty launch file
```
roslaunch catvehicle catvehicle_empty.launch
```
3) Launch points cloud to scan file
```
roslaunch catvehicle points_to_scan.launch
```
4) Run the gmapping launch file
```
roslaunch catvehicle nissan_gmapping.launch
```
