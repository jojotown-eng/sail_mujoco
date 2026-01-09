import mujoco
import numpy as np
import time
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("two_freedom_insect.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

model.opt.gravity[:] = [0, 0, 0]

motor_num = {
    'front_left_motor': 0,
    'front_left_motor2': 1,
    'front_right_motor': 2,
    'front_right_motor2': 3,
    'back_left_motor': 4,
    'back_left_motor2': 5,
    'back_right_motor': 6,
    'back_right_motor2': 7
}

# data.ctrl[0] = np.pi / 2

with viewer.launch_passive(model, data) as v:
	while (time.time() - t0 < duration) and v.is_running():
		t = time.time() - t0
		if t > 1.0:
			model.opt.gravity[:] = [0, 0, -9.81]
		if t > 3.0:
			
			data.ctrl[motor_num['front_left_motor']] = (np.pi / 6) * np.sin(2 * np.pi * t )
			data.ctrl[motor_num['front_left_motor2']] = (np.pi / 6) * np.sin(2 * np.pi * t )
			data.ctrl[2] = (np.pi / 6) * np.sin(2 * np.pi * t )
			data.ctrl[3] = (np.pi / 6) * np.sin(2 * np.pi * t )
		mujoco.mj_step(model, data)
		v.sync()