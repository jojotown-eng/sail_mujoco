import mujoco
import numpy as np
import time
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("insect.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

model.opt.gravity[:] = [0, 0, 0]

# data.ctrl[0] = np.pi / 2

with viewer.launch_passive(model, data) as v:
	while (time.time() - t0 < duration) and v.is_running():
		t = time.time() - t0
		if t > 1.0:
			model.opt.gravity[:] = [0, 0, -9.81]
		if t > 3.0:
			
			data.ctrl[0] = (np.pi / 6) * np.sin(2 * np.pi * t *0.5)
			data.ctrl[1] = -(np.pi / 6) * np.sin((2 * np.pi * t + np.pi*1.5) * 0.5)
		mujoco.mj_step(model, data)
		v.sync()