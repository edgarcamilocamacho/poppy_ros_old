#!/usr/bin/env python3
# Autor: Camilo Romero 


from getch import getch, pause
from IODynamixel.IODynamixel import IODynamixel
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from poppy_ROS.msg import Num
from poppy_ROS.msg import joints_seq
import rospy

dxl = None

def callback_play_movement(msg):
	global dxl, motor
	dxl.enableTorque(motor)
	name = msg.data
	rospy.loginfo(rospy.get_caller_id() + "The movement %s will be reproduce", msg.data)
	if name[-5:]!='.json':
		movement = dxl.loadMovement('./movements/' + name + ".json")
	else:
		movement = dxl.loadMovement('./movements/' + name)
	dxl.setMovementInit(movement)
	dxl.playMovement(movement)
def callback_set_angle(msg):
	global dxl
	rospy.loginfo(rospy.get_caller_id() + "Set new position")
	dxl.setAngle(msg.name, msg.position)    
def callback_close(msg):
	global dxl
	rospy.loginfo(rospy.get_caller_id() + "\nThe process has been stopped")
	dxl.stop()
def callback_rec_movement_start(msg):
	global dxl
	dxl.recordMovement(msg.name)
def callback_prueba(msg):
	global dxl
	print(msg.joints)
	print(msg.angles)
def callback_rec_movement_stop(msg):
	global dxl, motor
	mov = dxl.stopRecording()
	#dxl.saveMovement(mov,"/home/camilo_romero/catkin_ws/src/poppy_ROS/movements/" + msg.data + ".json")
	dxl.saveMovement(mov,"./movements/" + msg.data + ".json")
	print("The movement has been saved as " + msg.data + ".json")
	dxl.enableTorque(motor)
def listener():
	global dxl, motor
	#Using Vrep
	#dxl = IODynamixel(creature="./src/poppy_torso.json", simulator='vrep')
	dxl = IODynamixel(creature="./src/poppy_torso.json")
	dxl.start()
	motor = dxl.get_motor_names()
	dxl.enableTorque(motor)
	print("Init Node")
	rospy.init_node('listener', anonymous=True)
	print("Subscribing Topics")
	rospy.Subscriber("poppy_play_movement", String, callback_play_movement)
	rospy.Subscriber("poppy_rec_movement_stop", String, callback_rec_movement_stop)
	rospy.Subscriber("poppy_set_angle", JointState, callback_set_angle)
	rospy.Subscriber("poppy_rec_movement_start", JointState, callback_rec_movement_start)
	rospy.Subscriber("poppy_close", String, callback_close)
	rospy.Subscriber("poppy_prueba", joints_seq, callback_prueba)

	print("Waiting a message...\n")
	rospy.spin()

if __name__ == '__main__':
   listener()