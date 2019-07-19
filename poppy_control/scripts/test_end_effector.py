#!/usr/bin/env python2
import sys
import os
import rospy
import numpy as np
import time
import math
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotState

from IODynamixel.IODynamixel import read_creature
from poppy_control.srv import PlanMovement, PlanMovementResponse
from poppy_control.msg import Trajectory

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
    groups[name].set_planner_id('')
    groups[name].set_planning_time(2)
    groups[name].set_num_planning_attempts(10)
    groups[name].set_goal_tolerance(0.1)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
                                               moveit_msgs.msg.DisplayTrajectory, \
                                               queue_size=20)
#planning_frame = group.get_planning_frame()
#eef_link = group.get_end_effector_link()

motors = read_creature(creature="{}/creatures/poppy_torso_sim.json")

# groups['r_arm_4'].set_start_state_to_current_state()
# pos = groups['r_arm_4'].get_current_pose('r_hand').pose

pos = groups['r_arm_4'].get_current_joint_values()
print(pos)

pos = groups['r_arm_4'].get_current_pose().pose
print(pos)

# groups['r_arm_4'].set_position_target([	pos.position.x+0.1, 
# 										pos.position.y,
# 										pos.position.z],
# 										'r_hand')
# plan1 = groups['r_arm_4'].plan()
# groups['r_arm_4'].execute(plan1, wait=True)