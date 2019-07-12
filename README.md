# Poppy ROS Packages

This ROS packages allows using Poppy Robot with ROS (using de IODynamixel controller).

Do not forget to visit our web page: https://cstopics.github.io/cstopics/

## Dependencies

Tested with Ubuntu 16.04

* **Python == 2.7**

MoveIt didn't work with python 3.

* **IODynamixel**

Installation instructions: https://github.com/cstopics/IODynamixel

* **ROS Kinetic**

It was tested with ROS Kinetic: http://wiki.ros.org/. If you already have it installed, you should ensure it is updates as follows:

``` bash
$ sudo rm /etc/apt/sources.list.d/ros-latest.list
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

* ***MoveIt!*** 

https://moveit.ros.org/. Instructions:

``` bash
$ sudo apt-get install ros-${ROS_DISTRO}-catkin python-catkin-tools
$ sudo apt install ros-${ROS_DISTRO}-moveit
```

## Installation

Create a catkin workspace, clone the repository and make it:
``` bash
$ mkdir -p ~/ros/poppy_ws/src
$ cd ~/ros/poppy_ws/src/
$ git clone https://github.com/cstopics/poppy_ros
$ cd ..
$ catkin_make
```

## Nodes

### *poppy_control*

**Published topics**:
* /joint_states ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))

**Subscribed topics**:
* /poppy_control/playPredefMovement ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
* /execute_trajectory/goal (moveit_msgs/ExecuteTrajectoryActionGoal)

## Running

For **simulation**, don't forget starting *vrep* before running the nodes.

You can use the *launch* file to run all the nodes, or run each node in different terminals.

***Launch* file**:

``` bash
$ source ~/ros/poppy_ws/devel/setup.bash
$ roslaunch poppy_control poppy_control.launch
```

**Different terminals:**

Terminal 1, */poppy_control_dxl* node:

``` bash
$ source ~/ros/poppy_ws/devel/setup.bash
$ rosrun poppy_control poppy_control_dxl.py
```

Terminal 2, */poppy_control_moveit* node:

``` bash
$ source ~/ros/poppy_ws/devel/setup.bash
$ rosrun poppy_control poppy_control_moveit.py
```

Terminal 3, MoveIt nodes:

``` bash
$ source ~/ros/poppy_ws/devel/setup.bash
$ roslaunch poppy_moveit_config poppy.launch
```

## Testing

### Predefined movement

From terminal:

``` bash
rosservice call /poppy_predef_movement saludos
```

From python:

## Thanks!
