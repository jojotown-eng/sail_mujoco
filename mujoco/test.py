import mujoco
import numpy as np
from mujoco import viewer

xml = """
<mujoco>
  <worldbody>
    <body name="ball" pos="0 0 0">
      <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()