import rospy
import numpy as np
import time
import math
from poppy_control.srv import *
from poppy_control_utils import *

group = 'r_arm'
#target = [161.29, -65.42]
# target = [0.6482350826263428, 0.4289175271987915]
fps = 40.0

rospy.wait_for_service('/poppy_plan_movement')
plan_mov = rospy.ServiceProxy('/poppy_plan_movement', PlanMovement)

rospy.wait_for_service('/poppy_play_movement')
play_mov = rospy.ServiceProxy('/poppy_play_movement', PlayMovement)

resp = plan_mov(	group, 		# group
					1,			# angles_format
					False, 		# rand_start
					True,		# current_start 
					[],			# start_pos
					True,		# rand_target
					[],		# target_pos 
					False, 		# execute
					False, 		# wait
					True,		# ret_plan
					fps,		# ret_fps
					)

mov = plan2mov(resp, fps)

print('\n--- Movement ---')
print(mov)
print('\n--- Start Position ---')
print(resp.start_pos)
print('\n--- Target Position ---')
print(resp.target_pos)
print('\n--- Error ---')
print(resp.error)

if resp.error==0:
	resp = play_mov(	True,			# wait
						fps, 			# fps
						mov2traj(mov),	# trajectories
						)

	print('\n\n--- Error ---')
	print(resp.error)