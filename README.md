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

### *poppy_control_dxl*

**Published topics**:

* /joint_states ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))

Joints positions for moveit. Published at the *IODynamixel* frequency (40HZ by default).

* /poppy_joint_states ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))

Joints positions in degrees (with offsets and intertions applied, according to the *IODynamixel* creature). Published at the *IODynamixel* frequency (40HZ by default).

**Subscribed topics**:

* /execute_trajectory/goal (moveit_msgs/ExecuteTrajectoryActionGoal)

Message from *MoveIt* to execute trajectory.

**Services**:

* /poppy_predef_movement ([poppy_control/PredefMovement](https://github.com/cstopics/poppy_ros/blob/master/poppy_control/srv/PredefMovement.srv))

This service execute a predefined movement.

| Request  |  |
| ------------- | ------------- |
| *string* movement  | array of joints names  |
|  | *Included movements:* |
|  | "{}/movements/saludo.json" |
|  | *User movements:* |
|  | "/home/user1/movements/surpise.json" |
| *bool* wait  | wait until movement finishes |

| Reply |  |
| ------------- | ------------- |
| *int8* error  | *0* no error, *1* movement not found  |

* /poppy_play_movement ([poppy_control/PlayMovement](https://github.com/cstopics/poppy_ros/blob/master/poppy_control/srv/PlayMovement.srv))

This service receives and executes a movement

| Request |  |
| ------------- | ------------- |
| *bool* wait | Wait until movement finishes |
| *int8* fps | Frames per second of the movement |
| *Trajectory[]* trajectories | trajectory for each joint ([poppy_control/Trajectory](https://github.com/cstopics/poppy_ros/blob/master/poppy_control/msg/Trajectory.msg))|
            
| Reply |  | 
| ------------- | ------------- |
| *int8* error | *0*. No error |

* /poppy_goto_positions ([poppy_control/GotoPositions](https://github.com/cstopics/poppy_ros/blob/master/poppy_control/srv/GotoPositions.srv))

This service sets the position of the joints

| Request |  |
| ------------- | ------------- |
| *string[] joints*  | array of joints names |
| *float32[] positions* | desired positions |
            
| Reply |  |
| ------------- | ------------- |
| *int8* error | *0*. No error |

### *poppy_control_moteit*

**Services**:

* /poppy_predef_movement ([poppy_control/PlanMovement](https://github.com/cstopics/poppy_ros/blob/master/poppy_control/srv/PlanMovement.srv))

Plan, execute and return a trajectory, from the current/a random/a specified position to a random/specified target.

| Request |  |
| ------------- | ------------- |
| *string* group | group name, listed below |
| *int8* angles_format | angles format (for target, start and returned plan): *0* radians motor referenced, *1* degrees creature referenced |
| *bool* rand_start | random start position |
| *bool* current_start | set current position as start position (does not have effect if *rand_start==True*) |
| *float32[]* start_pos | start position (could be empty if *rand_start==True* or *current_start==True*) |
| *bool* rand_target | random target position |
| *float32[]* target_pos | target position (could be empty if *rand_target==True*) |
| *bool* execute | *False*, only plans. *True*, plans and executes |
| *bool* wait | wait until execution finishes (does not have effect if *execute==False*) |
| *bool* ret_plan | return calculated plan |
| *int8* ret_fps | frames per second for returned plan (does not have effect if *ret_plan==False*) |

| Reply |  |
| ------------- | ------------- |
| *int8* error | error, listed below |
| *float32[]* start_pos | start position |
| *float32[]* target_pos | target position |
| *Trajectory[]* plans | trajectory for each joint ([poppy_control/Trajectory](https://github.com/cstopics/poppy_ros/blob/master/poppy_control/msg/Trajectory.msg)) |

Avaliable groups: 

| Group | Joints |
| ------------- | ------------- |
| *all* | abs_z, bust_y, bust_x, head_z, head_y, l_shoulder_y, l_shoulder_x, l_arm_z, l_elbow_y, r_shoulder_y, r_shoulder_x, r_arm_z, r_elbow_y |
| *head* | head_z, head_y |
| *l_arm* | l_shoulder_y, l_shoulder_x, l_arm_z, l_elbow_y |
| *r_arm* | r_shoulder_y, r_shoulder_x, r_arm_z, r_elbow_y |
| *r_arms* | r_shoulder_y, r_shoulder_x, r_arm_z, r_elbow_y, l_shoulder_y, l_shoulder_x, l_arm_z, l_elbow_y |
| *torso* | bust_x, bust_y, abs_z |
| *r_arm_3* | r_upper_arm, r_forearm, r_shoulder |
| *r_arm_2* | r_forearm, r_shoulder |

Errors: 

| Error | Reason |
| ------------- | ------------- |
| *0* | no error |
| *1* | incorrect group name |
| *2* | could not plan |
| *3* | incorrect target positions |

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

Terminal 2, MoveIt nodes:

``` bash
$ source ~/ros/poppy_ws/devel/setup.bash
$ roslaunch poppy_moveit_config poppy.launch
```

Terminal 3, */poppy_control_moveit* node:

``` bash
$ source ~/ros/poppy_ws/devel/setup.bash
$ rosrun poppy_control poppy_control_moveit.py
```


## Testing

### Predefined movement

From terminal, waiting until movement finishes:

``` bash
rosservice call /poppy_predef_movement "{movement: '{}/movements/saludo.json', wait: True}"
```

From terminal, without waiting until movement finishes:

``` bash
rosservice call /poppy_predef_movement "{movement: '{}/movements/saludo.json', wait: False}"
```

From python:



## Thanks!
