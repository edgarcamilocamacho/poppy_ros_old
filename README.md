# Poppy ROS Packages

This ROS packages allows using Poppy Robot with ROS (using de IODynamixel controller).

Do not forget to visit our web page: https://cstopics.github.io/cstopics/

## Dependencies

* Python >= X.X
* IODynamixel (https://github.com/cstopics/IODynamixel)
* ROS Kinetic (http://www.ros.org/)
* MoveIt! (https://moveit.ros.org/) (Optional)

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

## Testing

Setup the workspace:
``` bash
$ cd ~/ros/poppy_ws/
$ source devel/setup.bash
```

### *poppy_control* node

Run the *poppy_control* node:
``` bash
$ rosrun poppy_control poppy_control.py
```
Play predefined movement:
``` bash
$ rostopic pub /poppy_control/playPredefMovement std_msgs/String "saludo"
```

## Thanks!
