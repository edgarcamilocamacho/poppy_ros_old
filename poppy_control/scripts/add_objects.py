import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.pose.position.x = 0.3
box_pose.pose.position.y = 0.3
box_pose.pose.position.z = 0.3
print(box_pose)
box_pose.header.frame_id = "/base"
box_pose.pose.orientation.w = 1.0
box_name = "box"
scene.add_box(box_name, box_pose, size=(1, 1, 1))