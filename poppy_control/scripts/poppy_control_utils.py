from poppy_control.msg import Trajectory

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