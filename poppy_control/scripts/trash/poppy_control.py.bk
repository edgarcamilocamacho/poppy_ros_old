#!/usr/bin/env python3
import sys
import os
import rospy
import json
import numpy as np
import time
import matplotlib.pyplot as plt
import signal

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from IODynamixel.IODynamixel import IODynamixel

def pubCallback(motors):
    msg = JointState()
    t = time.time()
    msg.header.stamp.secs = int(np.floor(t))
    msg.header.stamp.nsecs = int(np.floor((t-np.floor(t))*1000000000))
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

def callbackTrajectory(data):
    rospy.loginfo(rospy.get_caller_id() + "Trajectory received from MoveIt!")
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

def callbackPredefMovement(data):
    # rostopic pub /poppy_control/playPredefMovement std_msgs/ring 'macarena'
	name = data.data
	rospy.loginfo(rospy.get_caller_id() + "Executing movement: '" + name + "'")
	if name[-5:]!='.json':
		movement = dxl.loadMovement('{}/movements/' + name + ".json")
	else:
		movement = dxl.loadMovement('{}/movements/' + name)
	dxl.setMovementInit(movement)
	time.sleep(0.2)
	dxl.playMovement(movement)

def callbackSetPosition(data):
    # rostopic pub /poppy_control/setPosition sensor_msgs/JointState '{name:['r_shoulder_x'], position:[10.0]}'
    rospy.loginfo(rospy.get_caller_id() + "Going to position")
    dxl.setAngle(data.name, data.position)

print('Starting node...')
rospy.init_node('poppy_control', anonymous=False)
print('Creating publishers...')
pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
print('Creating Subscribers...')
rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, callbackTrajectory)
rospy.Subscriber("/poppy_control/playPredefMovement", String, callbackPredefMovement)
rospy.Subscriber("/poppy_control/setPosition", JointState, callbackSetPosition)

print('Creating IODynamixel controller...')
dxl = IODynamixel(creature="{}/creatures/poppy_torso_sim.json", simulator='vrep')

if not dxl.correct:
    print('Error creating IODynamixel object')
    os._exit(1)

dxl.setCallbackPost(pubCallback)

def signal_handler(sig, frame):
        print('\nStopping thread')
        dxl.stop()
        time.sleep(0.5)
        print('Halting...')
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

dxl.start()

print('Running...')
#rospy.spin()
