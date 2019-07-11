import sys
import numpy as np
import copy
import time
import json
import rospy
import pickle
from math import pi

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg._PoseStamped import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

NUM_SAMPLES = 120

targets2 = [ [0.6482350826263428, 0.4289175271987915],
            [-1.0356764793395996, 1.2443350553512573],
            [-0.3519880771636963, 0.9689768552780151],
            [-2.281643867492676, 0.7925108671188354],
            [0.7348535060882568, 2.309084415435791]]

targets3 = [[-1.872255563735962, 0.7731797695159912, 2.537353038787842],
            [0.3416016101837158, -1.298844337463379, 1.7428900003433228],
            [0.836427927017212, -0.5575861930847168, 0.38983309268951416],
            [-0.17501521110534668, -1.760706663131714, 1.4732178449630737],
            [-1.5256600379943848, 0.17832946777343753, 0.9654353857040405]]

targets4 = [[-2.541459798812866, 1.3173927068710327, 1.323833703994751, 1.6430891752243042],
            [0.0955967903137207, 1.0249346494674683, 0.9669079780578614, 1.2190724611282349],
            [-1.1097209453582764, 0.05950629711151123, -1.4632620811462402, 0.938045859336853],
            [-0.6595399379730225, -0.30261838436126715, -0.4938545227050781, 2.339563369750977],
            [-0.10565876960754396, 1.3991609811782837, -1.0498003959655762, 0.9725700616836548]]

targets = {"r_arm":targets4, "r_arm_3":targets3, "r_arm_2":targets2}

motors = {}
creature = "/home/camilo/repositories/IODynamixel/creatures/poppy_torso_sim.json"
with open(creature) as f:
    motors = json.load(f)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test_moveit', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "r_arm_2"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_planner_id("")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
                                               moveit_msgs.msg.DisplayTrajectory, \
                                               queue_size=20)
planning_frame = group.get_planning_frame()
eef_link = group.get_end_effector_link()
group_names = robot.get_group_names()

group.set_planning_time(5)

def planWithTime():
    start = time.time()
    plan = group.plan()
    stop = time.time()
    print('PLanning time: ' + str(stop-start))
    return plan

def goToRandom():
    global group
    while True:
        group.set_random_target()
        plan = group.plan()
        if len(plan.joint_trajectory.points)>0:
            break
    group.execute(plan, wait=True)
    time.sleep(0.5)

def goToPos(target):
    global group
    while True:
        group.set_joint_value_target(target)
        plan = group.plan()
        if len(plan.joint_trajectory.points)>0:
                break
        goToRandom()
    group.execute(plan, wait=True)
    time.sleep(0.5)
    return plan

def interpolate(route):
    global motors
    freq = 40.0
    period = 1/freq
    times = np.array(route['times'])
    times_new = np.arange(times.min(), times.max()+period, period)
    movement = {}
    movement['fps'] = freq
    movement['data'] = {}
    for motor in route['motors']:
        if motor=='r_shoulder_x':
            data = np.degrees(-np.interp(times_new, times, np.array(route['motors'][motor])))
        else:
            data = np.degrees(np.interp(times_new, times, np.array(route['motors'][motor])))
        if motors[motor]['invert']:
            movement['data'][motor] = list(-(data - motors[motor]['offset']))
        else:
            movement['data'][motor] = list(data - motors[motor]['offset'])
        #movement['data'][motor] = list(np.degrees(np.interp(times_new, times, np.array(route['motors'][motor]))))
    return movement

def planToNumpy(plan):
    rospy.loginfo(rospy.get_caller_id() + "Trajectory")
    traj = plan.joint_trajectory
    route = {}
    route['names'] = traj.joint_names
    route['times'] = len(traj.points)*[0]
    route['motors'] = {}
    for i in range(len(traj.points)):
        route['times'][i] = traj.points[i].time_from_start.secs + traj.points[i].time_from_start.nsecs/1000000000.0
    for m, motor in enumerate(traj.joint_names):
        route['motors'][motor] = len(traj.points)*[0]
        for i in range(len(traj.points)):
            route['motors'][motor][i] = traj.points[i].positions[m]
    return interpolate(route)

print("")

joint_state = JointState()
moveit_robot_state = RobotState()
joint_state.name = group.get_joints()


all_set = []
for j, target in enumerate(targets[group_name]):
    mov_set = {}
    mov_set['target'] = target
    mov_set['movements'] = []
    group.set_joint_value_target(target)
    for i in range(NUM_SAMPLES):
        while True:
            joint_state.header.stamp = rospy.Time.now()
            t = group.get_random_joint_values()
            joint_state.position = t
            moveit_robot_state.joint_state = joint_state
            group.set_start_state(moveit_robot_state)
            plan = group.plan()
            if len(plan.joint_trajectory.points)>0:
                break
        movement = planToNumpy(plan)
        mov_set['movements'].append(movement)
        print('!!! Sample:{}/{} - Target {}/{} - Finished'.format(i+1, NUM_SAMPLES, j+1, len(targets[group_name])))
    all_set.append(mov_set)

f = open('data_'+group_name+'.pkl', 'wb')
pickle.dump(all_set, f)
f.close()

print("finish")