import sys
import rospy
from poppy_control.srv import *

rospy.wait_for_service('/poppy_plan_movement')

predef_mov = rospy.ServiceProxy('/poppy_plan_movement', PlanMovement)
resp = predef_mov(	'r_arm_2', 	# group
					1,			# angles_format
					False, 		# rand_start
					False,		# current_start 
					# [1.2443350553512573, 1.2443350553512573],			# start_pos
					[161.29, -18.70],			# start_pos
					False,		# rand_target
					# [1.2443350553512573, 0.4289175271987915],			# target_pos 
					[161.29, -65.42],			# target_pos 
					False, 		# execute
					False, 		# wait
					True,		# ret_plan
					40)			# ret_fps
print(resp)
