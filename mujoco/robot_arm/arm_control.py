import mujoco
import numpy as np
import time
from mujoco import viewer

def VelocityControl(now_vel, goal_vel):
	now_error = goal_vel - now_vel
	output_value = 0.5*now_error + pre_error*0.01
	pre_error = now_error
	return output_value

def lerp(a, b, t):
    return a + (b - a) * t

model = mujoco.MjModel.from_xml_path("./arx_l5/scene.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

model.opt.gravity[:] = [0, 0, -9.81]

state = 1

# data.ctrl[0] = np.pi / 2

with viewer.launch_passive(model, data) as v:
	while (time.time() - t0 < duration) and v.is_running():
		t = time.time() - t0
		if(t < 2):
			q_target = lerp(0, np.pi/2, t / 2)
			data.ctrl[0] = q_target
		# if(t < 6):
		# 	q_target = lerp(0, np.pi/4, t / 6)
		# 	data.ctrl[1] = q_target
		# q_target = lerp(0, np.pi/4, t / 4)
		# data.ctrl[2] = q_target
		# print(q_target)
		mujoco.mj_step(model, data)
		v.sync()