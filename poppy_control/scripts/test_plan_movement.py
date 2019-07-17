import sys
import rospy
from poppy_control.srv import *

rospy.wait_for_service('/poppy_plan_movement')

try:
    predef_mov = rospy.ServiceProxy('/poppy_plan_movement', PlanMovement)
    for _ in range(3):
        resp = predef_mov(  'arms',  # group
                            1,          # angles_format
                            False,      # rand_start
                            True,       # current_start 
                            # [1.2443350553512573, 1.2443350553512573],         # start_pos
                            # [161.29, -18.70],         # start_pos
                            [],
                            True,       # rand_target
                            # [1.2443350553512573, 0.4289175271987915],         # target_pos 
                            # [161.29, -65.42],         # target_pos 
                            [],
                            True,       # execute
                            True,       # wait
                            True,       # ret_plan
                            40)         # ret_fps
        print('response: ', resp)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e