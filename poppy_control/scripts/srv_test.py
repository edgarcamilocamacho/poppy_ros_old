import sys
import rospy
from poppy_control.srv import *

def movement(mov):
    rospy.wait_for_service('/poppy_predef_movement')
    try:
        predef_mov = rospy.ServiceProxy('poppy_predef_movement', PredefMovement)
        resp = predef_mov(mov)
        return resp.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

print( movement('saludo') )