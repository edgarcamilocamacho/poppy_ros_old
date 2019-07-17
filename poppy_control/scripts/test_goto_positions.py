import sys
import rospy
import time
from poppy_control.srv import *

rospy.wait_for_service('/poppy_goto_positions')
try:
    goto_pos = rospy.ServiceProxy('/poppy_goto_positions', GotoPositions)
    for _ in range(3):
        resp = goto_pos(['r_shoulder_y', 'l_shoulder_y', 'l_arm_z'],[10.0, 20.0, 30.0])
        print(resp)
        time.sleep(1)
        resp = goto_pos(['r_shoulder_y', 'l_shoulder_y', 'l_arm_z'],[0.0, 0.0, 0.0])
        print(resp)
        time.sleep(1)    
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

