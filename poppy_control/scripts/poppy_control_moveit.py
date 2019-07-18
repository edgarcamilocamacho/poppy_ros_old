#!/usr/bin/env python2
import sys
import os
import rospy
import numpy as np
import time
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotState

from IODynamixel.IODynamixel import read_creature
from poppy_control.srv import PlanMovement, PlanMovementResponse, GetEndEffectorPos, GetEndEffectorPosResponse
from poppy_control.msg import Trajectory

# rosservice call /poppy_predef_movement saludos

### CALLBACKS
def srvPlanMovementCallback(data):
    group_name = data.group
    if group_name in groups_names:
        
        if data.rand_target:
            target_position = groups[group_name].get_random_joint_values()
        else:
            if data.angles_format == 0:
                target_position = data.target_pos
            else:
                target_position = robot2rad(groups[group_name], data.target_pos)
        try:
            groups[group_name].set_joint_value_target(target_position)
        except:
            rospy.logwarn(rospy.get_caller_id() + ' Incorrect target positions')
            return PlanMovementResponse(3, [], [], [])
        rospy.loginfo(rospy.get_caller_id() + " Moving group '" + group_name + "' to " + str(data.target_pos) )
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.stamp.secs += 1
        joint_state.name = groups[group_name].get_active_joints()
        if data.rand_start:
            start_position = groups[group_name].get_random_joint_values()
        else:
            if data.current_start:
                start_position = groups[group_name].get_current_joint_values()   
            else:
                if data.angles_format == 0:
                    start_position = data.start_pos
                else:
                    start_position = robot2rad(groups[group_name], data.start_pos)
        joint_state.position = start_position                                
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        groups[group_name].set_start_state(moveit_robot_state)
        plan = groups[group_name].plan()
        if len(plan.joint_trajectory.points)>0:
            rospy.loginfo(rospy.get_caller_id() + ' Correct plan')
            if data.execute:
                if data.wait:
                    rospy.logwarn(rospy.get_caller_id() + ' WAIT')
                    groups[group_name].execute(plan, wait=True)
                else:
                    rospy.logwarn(rospy.get_caller_id() + ' NO WAIT')
                    groups[group_name].execute(plan, wait=False)
            if data.ret_plan:
                traj = planToNumpy(plan, data.ret_fps, data.angles_format)
                plans = []
                for joint in traj['data']:
                    plans.append(Trajectory(joint, traj['data'][joint]))
                if data.angles_format == 0:
                    return PlanMovementResponse(0,  start_position, 
                                                    target_position, 
                                                    plans)
                else:
                    return PlanMovementResponse(0,  rad2robot(groups[group_name], start_position), 
                                                    rad2robot(groups[group_name], target_position), 
                                                    plans)
            else:
                return PlanMovementResponse(0, [], [], [])
        else:
            rospy.logwarn(rospy.get_caller_id() + ' Could not plan')
            return PlanMovementResponse(2, [], [], [])
    else:
        rospy.logwarn(rospy.get_caller_id() + ' Incorrect group name: ' + group_name)
        return PlanMovementResponse(1, [], [], [])

def srvGetEndEffectorPosCallback(data):
    pos = groups[data.group].get_current_pose().pose
    return GetEndEffectorPosResponse([pos.position.x, pos.position.y, pos.position.z])

### GLOBAL FUNCTIONS

def robot2rad(group, angles):
    joint_names = group.get_active_joints()
    new_angles = [0] * len(joint_names)
    for i, joint in enumerate(joint_names):
        new_angles[i] = math.radians(angles[i] + motors[joint]['offset'])
    return new_angles

def rad2robot(group, angles):
    joint_names = group.get_active_joints()
    new_angles = [0] * len(joint_names)
    print('joint_names', joint_names)
    print('angles', angles)
    print('new_angles', new_angles)
    for i, joint in enumerate(joint_names):
        new_angles[i] = math.degrees(angles[i]) - motors[joint]['offset']
    return new_angles

def interpolate(route, freq, angles_format):
    global motors
    period = 1/float(freq)
    times = np.array(route['times'])
    times_new = np.arange(times.min(), times.max()+period, period)
    movement = {}
    movement['fps'] = freq
    movement['data'] = {}
    if angles_format==0:
        for motor in route['motors']:
            data = np.interp(times_new, times, np.array(route['motors'][motor]))
            movement['data'][motor] = list(data)
    else:
        for motor in route['motors']:
            if motor=='r_shoulder_x':
                data = np.degrees(-np.interp(times_new, times, np.array(route['motors'][motor])))
            else:
                data = np.degrees(np.interp(times_new, times, np.array(route['motors'][motor])))
            if motors[motor]['invert']:
                movement['data'][motor] = list(-(data - motors[motor]['offset']))
            else:
                movement['data'][motor] = list(data - motors[motor]['offset'])
            #movement['data'][motor] = list(np.degrees(np.interp(times_new, times, np.array(route['motors'][motor]))))
    return movement

def planToNumpy(plan, freq, angles_format):
    rospy.loginfo(rospy.get_caller_id() + "Trajectory")
    traj = plan.joint_trajectory
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
    return interpolate(route, freq, angles_format)

### MAIN

node_name = 'poppy_control_moveit'

rospy.loginfo(rospy.get_caller_id() + ' Node created')
rospy.init_node(node_name, anonymous=False)
rospy.loginfo(rospy.get_caller_id() + ' Connecting with moveit...')
moveit_commander.roscpp_initialize(sys.argv)
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

motors = read_creature(creature="{}/creatures/poppy_torso_sim.json")

#rospy.loginfo(rospy.get_caller_id()+ ' Creating publishers...')
###
#rospy.loginfo(rospy.get_caller_id()+ ' Creating subscribers...')
###
#rospy.loginfo(rospy.get_caller_id()+ ' Creating services...')
srv_plan_movement = rospy.Service('/poppy_plan_movement', PlanMovement, srvPlanMovementCallback)
srv_get_end_effector_pos = rospy.Service('/poppy_get_end_effector_pos', GetEndEffectorPos, srvGetEndEffectorPosCallback)

rospy.loginfo(rospy.get_caller_id() + ' Ready!')

rospy.spin()


