import time
import numpy
import pypot.robot

poppy = pypot.robot.from_json('test.json')

while(1):
	time.sleep(0.02)
	print(poppy.r_elbow_y.present_position)