import rospy
import numpy as np
import time
import math
import os

from poppy_control.srv import *
from poppy_control_utils import *

NUM_SAMPLES = 1000
FPS = 40.0
GROUP = 'r_arm_4'
USER = os.environ['USER']
PATH = '/home/'+USER+'/Dropbox/Compartido/linux/poppy/expert_trajectories/expert_1.npy'

rospy.wait_for_service('/poppy_plan_movement')
plan_mov = rospy.ServiceProxy('/poppy_plan_movement', PlanMovement)
rospy.wait_for_service('/poppy_get_end_effector_pos')
get_end_effector = rospy.ServiceProxy('/poppy_get_end_effector_pos', GetEndEffectorPos)

movs = []
part = 0

for i in range(NUM_SAMPLES):
    print('sample {} of {}'.format(i, NUM_SAMPLES))
    resp = plan_mov(  GROUP,        # group
        1,                      # angles_format
        False,                   # rand_start
        True,                  # current_start 
        [],                     # start_pos
        True,                  # rand_target
        [],                     # target_pos 
        True,                  # execute
        True,                  # wait
        True,                   # ret_plan
        FPS,   # ret_fps
        )
    if resp.error==0:
        mov = plan2mov(resp, FPS)
        mov['start_angles'] = list(resp.start_pos)
        mov['target_angles'] = list(resp.target_pos)
        end_pos = get_end_effector(GROUP)
        mov['target_end_effector'] = list(end_pos.xyz)
        movs.append( mov )
        time.sleep(0.5)
np.save(PATH, movs)
