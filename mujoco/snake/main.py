import mujoco
import numpy as np
import time
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("snake.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

model.opt.gravity[:] = [0, 0, 0]

# data.ctrl[0] = np.pi / 2

amp = 0.6
omega = 4.0
phases = [0, 0.8, 1.6, 2.4]

with viewer.launch_passive(model, data) as v:
	while (time.time() - t0 < duration) and v.is_running():
		t = time.time() - t0
		if t > 1.0:
			model.opt.gravity[:] = [0, 0, -9.81]
		if t > 3.0:
			for i, ph in enumerate(phases):
				data.ctrl[0] = amp * np.sin(omega * t + ph)
			# data.ctrl[0] = 1.0
		mujoco.mj_step(model, data)
		v.sync()