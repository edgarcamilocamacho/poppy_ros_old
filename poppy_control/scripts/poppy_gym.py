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
from poppy_control.srv import GymStep, GymReset

### GLOBAL VARIABLES

group = ''
angles_format = 0

### CALLBACKS

def srvGymResetCallback(data):
	group = data.group
	angles_format = data.angles_format

def srvGymStepCallback(data):
	pass

### MAIN

node_name = 'poppy_gym'

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

motors = read_creature(creature="{}/creatures/poppy_torso_sim.json")

srv_rst = rospy.Service('/poppy_gym_reset', PlanMovement, srvGymResetCallback)
srv_step = rospy.Service('/poppy_gym_step', PlanMovement, srvGymStepCallback)

rospy.loginfo(rospy.get_caller_id() + ' Ready!')

rospy.spin()
