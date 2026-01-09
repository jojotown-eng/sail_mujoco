import mujoco
import numpy as np
import time
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("one_leg.xml")
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
			pass
		mujoco.mj_step(model, data)
		v.sync()