import sys
import rospy
from poppy_control.srv import *

rospy.wait_for_service('/poppy_goto_positions')

goto_pos = rospy.ServiceProxy('/poppy_goto_positions', GotoPositions)
resp = goto_pos(['r_shoulder_y', 'l_shoulder_y', 'l_arm_z'],[10.0, 20.0, 30.0])
print(resp)
