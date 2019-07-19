import rospy
import numpy as np
import time
import math
import os

from poppy_control.srv import *
import poppy_control_utils as pu

USER = os.environ['USER']
PATH = 		'/home/'+USER+'/Dropbox/Compartido/linux/poppy/expert_trajectories/expert_1.npy'
PATH_OUT = 	'/home/'+USER+'/Dropbox/Compartido/linux/poppy/expert_trajectories/expert_1_xyz.npy'

JOINTS = ['r_shoulder_y', 'r_shoulder_x', 'r_arm_z', 'r_elbow_y']

data = np.load(PATH)

rospy.wait_for_service('/poppy_forward_kinematics')
get_kinematics = rospy.ServiceProxy('/poppy_forward_kinematics', ForwardKinematics)

pu.load_dxl_model()

for c, mov in enumerate(data):
	print('Mov ' + str(c) + ' of ' + str(len(data)))
	traj_end_effector = []
	for i, _ in enumerate(mov['data'][JOINTS[0]]):
		pos = ['']*len(JOINTS)
		for j, joint in enumerate(JOINTS):
			pos[j] = mov['data'][joint][i]
		pos = pu.robot2rad(JOINTS, pos)
		xyz = get_kinematics('r_arm_4', pos).end_pos
		traj_end_effector.append(xyz)
	data[c]['end_effector_pos'] = traj_end_effector
	# break

np.save(PATH_OUT, data)