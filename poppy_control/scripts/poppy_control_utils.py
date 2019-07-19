from poppy_control.msg import Trajectory
from IODynamixel.IODynamixel import read_creature
import math

motors = None

def plan2mov(message, fps):
    mov = {}
    mov['fps'] = fps
    mov['data'] = {}
    for plan in message.plans:
        mov['data'][plan.joint] = plan.trajectory
    return mov

def mov2traj(mov):
    trajectories = []
    for joint in mov['data']:
        trajectories.append( Trajectory(joint, mov['data'][joint]) )
    return trajectories

def play2mov(message):
    mov = {}
    mov['fps'] = float(message.fps)
    mov['data'] = {}
    for plan in message.trajectories:
        mov['data'][plan.joint] = plan.trajectory
    return mov

def load_dxl_model(sim=True):
    global motors
    if sim:
        motors = read_creature(creature="{}/creatures/poppy_torso_sim.json")
    else:
        motors = read_creature(creature="{}/creatures/poppy_torso.json")

def robot2rad(joint_names, angles):
    global motors
    new_angles = [0] * len(joint_names)
    for i, joint in enumerate(joint_names):
        new_angles[i] = math.radians(angles[i] + motors[joint]['offset'])
    return new_angles

def rad2robot(joint_names, angles):
    global motors
    new_angles = [0] * len(joint_names)
    print('joint_names', joint_names)
    print('angles', angles)
    print('new_angles', new_angles)
    for i, joint in enumerate(joint_names):
        new_angles[i] = math.degrees(angles[i]) - motors[joint]['offset']
    return new_angles