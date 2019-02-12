#!/usr/bin/env python3
import rospy
import json
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal

jointState = []
trajectory = []

def callbackJointState(data):
    global jointState
    #rospy.loginfo(rospy.get_caller_id() + "JointState")
    jointState = data

def callbackTrajectory(data):
    global trajectory
    rospy.loginfo(rospy.get_caller_id() + "Trajectory")
    trajectory = data

def saveJointState(fileName):
    data = {'names':list(jointState.name), 'positions':list(jointState.position)}
    with open(fileName, 'w') as fp:
        json.dump(data, fp)

    
def listener():
    rospy.init_node('poppy_control', anonymous=False)

    rospy.Subscriber("/joint_states", JointState, callbackJointState)
    rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, callbackTrajectory)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()