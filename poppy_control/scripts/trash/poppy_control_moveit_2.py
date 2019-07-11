#!/usr/bin/env python2
import sys
import os
import rospy
import json
import numpy as np
import time
import matplotlib.pyplot as plt
import signal

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from IODynamixel.IODynamixel import IODynamixel
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from poppy_control.srv import *

# rosservice call /poppy_predef_movement saludos

### CALLBACKS

def pubJointStatesCallback(motors):
    msg = JointState()
    t = time.time()
    msg.header.stamp = rospy.Time.now()
    # msg.header.stamp.secs = int(np.floor(t))
    # msg.header.stamp.nsecs = int(np.floor((t-np.floor(t))*1000000000))
    names = []
    positions = []
    for motor in motors:
        names.append(motor)
        if motor=='r_shoulder_x':
            positions.append(np.radians(-motors[motor]['motorAngle']))
        else:
            positions.append(np.radians(motors[motor]['motorAngle']))
    msg.name = names
    msg.position = positions
    pub_joint_states.publish(msg)

def srvPredefMovementCallback(data):
    name = data.movement
    try:
        rospy.loginfo(rospy.get_caller_id() + " Executing movement: '" + name + "'")
        if name[-5:]!='.json':
            movement = dxl.loadMovement('{}/movements/' + name + ".json")
        else:
            movement = dxl.loadMovement('{}/movements/' + name)
        dxl.setMovementInit(movement)
        time.sleep(0.2)
        dxl.playMovementBlock(movement)
        rospy.loginfo(rospy.get_caller_id() + ' Movement execution finished')
        return PredefMovementResponse(True)
    except:
        rospy.logerr(rospy.get_caller_id() + ' Movement not executed')
        return PredefMovementResponse(False)

def subMoveitTrajectory(data):
    rospy.loginfo(rospy.get_caller_id() + " Trajectory received from MoveIt!")
    rospy.loginfo(rospy.get_caller_id() + " Executing trajectory...")
    traj = data.goal.trajectory.joint_trajectory
    route = {}
    route['names'] = traj.joint_names
    route['times'] = len(traj.points)*[0]
    route['motors'] = {}
    for i in range(len(traj.points)):
        route['times'][i] = traj.points[i].time_from_start.secs + traj.points[i].time_from_start.nsecs/1000000000.0
    for m, motor in enumerate(traj.joint_names):
        route['motors'][motor] = len(traj.points)*[0]
        for i in range(len(traj.points)):
            route['motors'][motor][i] = traj.points[i].positions[m]
    movement = dxl.interpolate(route)
    dxl.playMovementBlock(movement)
    rospy.loginfo(rospy.get_caller_id() + " Trajectory execution finished")

def srvPlanMovementCallback(data):
    group_name = data.group
    if group_name in groups_names:
        groups[group_name].set_joint_value_target(data.targets)
        rospy.loginfo(rospy.get_caller_id() + " Moving group '" + group_name + "' to " + str(data.targets) )
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.stamp.secs += 1
        joint_state.name = groups[group_name].get_joints()
        start_pos = groups[group_name].get_random_joint_values()
        print(start_pos)
        joint_state.position = start_pos
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        # moveit_robot_state.multi_dof_joint_state.header.stamp = rospy.Time.now()
        print(moveit_robot_state)
        groups[group_name].set_start_state(moveit_robot_state)
        #plan = group.plan()
        #print(plan)        
        return PlanMovementResponse(True, [])
    else:
        rospy.logerr(rospy.get_caller_id() + ' Incorrect group name: ' + group_name)
        return PlanMovementResponse(False, [])

### GLOBAL FUNCTIONS


### MAIN

node_name = 'poppy_control_moveit'

rospy.loginfo(rospy.get_caller_id() + ' Node created')
rospy.loginfo(rospy.get_caller_id() + ' Connecting with moveit...')
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node(node_name, anonymous=True)

robot = moveit_commander.RobotCommander()
groups_names = robot.get_group_names()
# groups_names = ['r_arm_2']
scene = moveit_commander.PlanningSceneInterface()
groups = {}
for name in groups_names:
    groups[name] = moveit_commander.MoveGroupCommander(name)
    groups[name].set_planner_id(name)
    groups[name].set_planning_time(5)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
                                               moveit_msgs.msg.DisplayTrajectory, \
                                               queue_size=20)
#planning_frame = group.get_planning_frame()
#eef_link = group.get_end_effector_link()

#rospy.loginfo(rospy.get_caller_id()+ ' Creating publishers...')
pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
#rospy.loginfo(rospy.get_caller_id()+ ' Creating subscribers...')
rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, subMoveitTrajectory)
#rospy.loginfo(rospy.get_caller_id()+ ' Creating services...')
srv_predef_movement = rospy.Service('/poppy_predef_movement', PredefMovement, srvPredefMovementCallback)
srv_plan_movement = rospy.Service('/poppy_plan_movement', PlanMovement, srvPlanMovementCallback)

rospy.loginfo(rospy.get_caller_id() + ' Creating IODynamixel controller...')
dxl = IODynamixel(creature="{}/creatures/poppy_torso_sim.json", simulator='vrep')
if not dxl.correct:
    rospy.logerr(rospy.get_caller_id() + ' Error creating IODynamixel object')
    #os._exit(1)
else:
    dxl.setCallbackPost(pubJointStatesCallback)
    rospy.loginfo(rospy.get_caller_id() + ' Starting IODynamixel controller...')
    dxl.start()
    rospy.loginfo(rospy.get_caller_id() + ' IODynamixel controller running')

    time.sleep(3)

    group_name = 'r_arm_2'
    groups[group_name].set_joint_value_target([0.6482350826263428, 0.4289175271987915])
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.header.stamp.secs += 1
    joint_state.name = groups[group_name].get_joints()
    start_pos = groups[group_name].get_random_joint_values()
    print(start_pos)
    joint_state.position = start_pos
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    # moveit_robot_state.multi_dof_joint_state.header.stamp = rospy.Time.now()
    print(moveit_robot_state)
    groups[group_name].set_start_state(moveit_robot_state)
    plan = group.plan()

    # rospy.spin(1)
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.1)

rospy.loginfo(rospy.get_caller_id() + ' Stopping thread')
dxl.stop()
time.sleep(0.5)


