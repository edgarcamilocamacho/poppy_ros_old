#!/usr/bin/env python2
import sys
import os
import rospy
import numpy as np
import time

# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from IODynamixel.IODynamixel import IODynamixel
# from moveit_msgs.msg import RobotState
# from sensor_msgs.msg import JointState

from poppy_control.srv import PredefMovement, PredefMovementResponse
from poppy_control.srv import PlayMovement, PlayMovementResponse
from poppy_control.srv import GotoPositions, GotoPositionsResponse
from poppy_control.msg import Trajectory

from poppy_control_utils import *

# TODO: Add FPS as parameter

# rosservice call /poppy_predef_movement saludo

### CALLBACKS

def pubJointStatesCallback(motors):
    msg = JointState()
    msg2 = JointState()
    msg.header.stamp = rospy.Time.now()
    msg2.header.stamp = rospy.Time.now()
    names = []
    positions = []
    positions2 = []
    for motor in motors:
        names.append(motor)
        if motor=='r_shoulder_x':
            positions.append(np.radians(-motors[motor]['motorAngle']))
        else:
            positions.append(np.radians(motors[motor]['motorAngle']))
        positions2.append(motors[motor]['robotAngle'])
    msg.name = names
    msg.position = positions
    msg2.name = names
    msg2.position = positions2
    pub_joint_states.publish(msg)
    pub_poppy_joint_states.publish(msg2)

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
        return PredefMovementResponse(0)
    except:
        rospy.logerr(rospy.get_caller_id() + ' Movement not executed')
        return PredefMovementResponse(1)

def srvPlayMovementCallback(data):
    mov = play2mov(data)
    rospy.loginfo(rospy.get_caller_id() + " Movement received")
    if data.wait:
        dxl.playMovementBlock(mov)
        rospy.loginfo(rospy.get_caller_id() + " Movement finished")
    else:
        dxl.playMovement(mov)
    return PlayMovementResponse(0)

def srvGotoPositionCallback(data):
    dxl.setAngle(data.joints, data.positions)
    return GotoPositionsResponse(0)

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



### GLOBAL FUNCTIONS


### MAIN

node_name = 'poppy_control_dxl'

rospy.loginfo(rospy.get_caller_id() + ' Node created')
rospy.init_node(node_name, anonymous=False)

#rospy.loginfo(rospy.get_caller_id()+ ' Creating publishers...')
pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
pub_poppy_joint_states = rospy.Publisher('/poppy_joint_states', JointState, queue_size=1)
#rospy.loginfo(rospy.get_caller_id()+ ' Creating subscribers...')
rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, subMoveitTrajectory)
#rospy.loginfo(rospy.get_caller_id()+ ' Creating services...')
srv_predef_movement = rospy.Service('/poppy_predef_movement', PredefMovement, srvPredefMovementCallback)
srv_play_movement = rospy.Service('/poppy_play_movement', PlayMovement, srvPlayMovementCallback)
srv_goto_position = rospy.Service('/poppy_goto_position', GotoPositions, srvGotoPositionCallback)

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
    rospy.loginfo(rospy.get_caller_id() + ' Ready!')
    rospy.spin()

rospy.loginfo(rospy.get_caller_id() + ' Stopping thread')
dxl.stop()
time.sleep(0.5)


