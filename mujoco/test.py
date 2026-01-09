import mujoco
import numpy as np
import time
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("robot_arm/arx_l5/scene.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

model.opt.gravity[:] = [0, 0, 0]

base_id = 0

# data.ctrl[0] = np.pi / 2

with viewer.launch_passive(model, data) as v:
	while (time.time() - t0 < duration) and v.is_running():
		t = time.time() - t0
		data.ctrl[int(base_id)] = np.pi*np.sin(2*np.pi*t*0.1)
		mujoco.mj_step(model, data)
		v.sync()