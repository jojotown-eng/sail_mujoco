import mujoco
import numpy as np
import time
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("insect.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

q_target = np.pi/2
kp = 2
kd = 1

with viewer.launch_passive(model, data) as v:
	while (time.time() - t0 < duration) and v.is_running():
		t = time.time() - t0
		mujoco.mj_step(model, data)
		v.sync()