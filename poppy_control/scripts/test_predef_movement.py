import sys
import rospy
from poppy_control.srv import *

mov_file = '{}/movements/saludo.json'
wait = False

rospy.wait_for_service('/poppy_predef_movement')
try:
    predef_mov = rospy.ServiceProxy('poppy_predef_movement', PredefMovement)
    resp = predef_mov(mov_file, wait)
    print(resp)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e