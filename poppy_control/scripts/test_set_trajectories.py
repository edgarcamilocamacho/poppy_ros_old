import rospy
import numpy as np
import time
import math
import os

from poppy_control.srv import *
from poppy_control_utils import *

NUM_SAMPLES = 120
USER = os.environ['USER']
PATH = '/home/'+USER+'/Dropbox/Compartido/linux/poppy/expert_trajectories'

experts = [ {'group': 'r_arm_2', 'fps': 40, 'target': [91.57, -55.54],  'file': 'expert_r_arm_2_40fps_1.npy' }, 
            {'group': 'r_arm_2', 'fps': 40, 'target': [-17.18, -56.78], 'file': 'expert_r_arm_2_40fps_2.npy' }, 
            {'group': 'r_arm_2', 'fps': 40, 'target': [201.84, -27.08], 'file': 'expert_r_arm_2_40fps_3.npy' }, 
            {'group': 'r_arm_2', 'fps': 40, 'target': [-1.75, 28.46],   'file': 'expert_r_arm_2_40fps_4.npy' }, 
            {'group': 'r_arm_2', 'fps': 40, 'target': [46.69, 29.40],   'file': 'expert_r_arm_2_40fps_5.npy' }, 

            {'group': 'r_arm_2', 'fps': 20, 'target': [91.57, -55.54],  'file': 'expert_r_arm_2_20fps_1.npy' }, 
            {'group': 'r_arm_2', 'fps': 20, 'target': [-17.18, -56.78], 'file': 'expert_r_arm_2_20fps_2.npy' }, 
            {'group': 'r_arm_2', 'fps': 20, 'target': [201.84, -27.08], 'file': 'expert_r_arm_2_20fps_3.npy' }, 
            {'group': 'r_arm_2', 'fps': 20, 'target': [-1.75, 28.46],   'file': 'expert_r_arm_2_20fps_4.npy' }, 
            {'group': 'r_arm_2', 'fps': 20, 'target': [46.69, 29.40],   'file': 'expert_r_arm_2_20fps_5.npy' }, 

            {'group': 'r_arm_3', 'fps': 40, 'target': [203.91, 20.12, -51.66],      'file': 'expert_r_arm_3_40fps_1.npy' }, 
            {'group': 'r_arm_3', 'fps': 40, 'target': [112.74, 6.43, 33.69],        'file': 'expert_r_arm_3_40fps_2.npy' }, 
            {'group': 'r_arm_3', 'fps': 40, 'target': [155.96, 88.55, 12.86],       'file': 'expert_r_arm_3_40fps_3.npy' }, 
            {'group': 'r_arm_3', 'fps': 40, 'target': [-55.01, -73.99, 7.62],       'file': 'expert_r_arm_3_40fps_4.npy' }, 
            {'group': 'r_arm_3', 'fps': 40, 'target': [77.96, -1.90, -13.82],       'file': 'expert_r_arm_3_40fps_5.npy' },

            {'group': 'r_arm_3', 'fps': 20, 'target': [203.91, 20.12, -51.66],      'file': 'expert_r_arm_3_20fps_1.npy' }, 
            {'group': 'r_arm_3', 'fps': 20, 'target': [112.74, 6.43, 33.69],        'file': 'expert_r_arm_3_20fps_2.npy' }, 
            {'group': 'r_arm_3', 'fps': 20, 'target': [155.96, 88.55, 12.86],       'file': 'expert_r_arm_3_20fps_3.npy' }, 
            {'group': 'r_arm_3', 'fps': 20, 'target': [-55.01, -73.99, 7.62],       'file': 'expert_r_arm_3_20fps_4.npy' }, 
            {'group': 'r_arm_3', 'fps': 20, 'target': [77.96, -1.90, -13.82],       'file': 'expert_r_arm_3_20fps_5.npy' },

            {'group': 'r_arm', 'fps': 40, 'target': [80.95, 77.19, -58.92, -88.49],       'file': 'expert_r_arm_40fps_1.npy' }, 
            {'group': 'r_arm', 'fps': 40, 'target': [60.09, 138.92, 53.08, 37.27],        'file': 'expert_r_arm_40fps_2.npy' }, 
            {'group': 'r_arm', 'fps': 40, 'target': [5.15, 20.25, 5.24, -47.68],          'file': 'expert_r_arm_40fps_3.npy' }, 
            {'group': 'r_arm', 'fps': 40, 'target': [-53.86, 135.90, -34.36, 43.37],      'file': 'expert_r_arm_40fps_4.npy' }, 
            {'group': 'r_arm', 'fps': 40, 'target': [68.32, 164.56, -98.16, 23.97],       'file': 'expert_r_arm_40fps_5.npy' },

            {'group': 'r_arm', 'fps': 20, 'target': [80.95, 77.19, -58.92, -88.49],       'file': 'expert_r_arm_20fps_1.npy' }, 
            {'group': 'r_arm', 'fps': 20, 'target': [60.09, 138.92, 53.08, 37.27],        'file': 'expert_r_arm_20fps_2.npy' }, 
            {'group': 'r_arm', 'fps': 20, 'target': [5.15, 20.25, 5.24, -47.68],          'file': 'expert_r_arm_20fps_3.npy' }, 
            {'group': 'r_arm', 'fps': 20, 'target': [-53.86, 135.90, -34.36, 43.37],      'file': 'expert_r_arm_20fps_4.npy' }, 
            {'group': 'r_arm', 'fps': 20, 'target': [68.32, 164.56, -98.16, 23.97],       'file': 'expert_r_arm_20fps_5.npy' },
            ]

rospy.wait_for_service('/poppy_plan_movement')
plan_mov = rospy.ServiceProxy('/poppy_plan_movement', PlanMovement)

for expert in experts:
    movs = []
    print('Creating file', expert['file'])
    for i in range(NUM_SAMPLES):
        print('  sample', i+1, 'of', NUM_SAMPLES)
        resp = plan_mov(  expert['group'],        # group
            1,                      # angles_format
            True,                   # rand_start
            False,                  # current_start 
            [],                     # start_pos
            False,                  # rand_target
            expert['target'],       # target_pos 
            False,                   # execute
            False,                   # wait
            True,                   # ret_plan
            float(expert['fps']),   # ret_fps
            )
        mov = plan2mov(resp, float(expert['fps']))
        mov['origin'] = list(resp.start_pos)
        mov['target'] = list(resp.target_pos)
        movs.append( mov )
    np.save(PATH + '/' + expert['file'], movs)

print(mov)