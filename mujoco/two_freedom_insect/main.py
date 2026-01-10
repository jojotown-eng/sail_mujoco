import mujoco
import numpy as np
import time
from mujoco import viewer
import cma

model = mujoco.MjModel.from_xml_path("two_freedom_insect.xml")
data = mujoco.MjData(model)

duration = 80.0
t0 = time.time()

# モーター番号
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

# 配列として格納する
motor_ids = np.array([
    motor_num['front_left_motor'],
    motor_num['front_left_motor2'],
    motor_num['front_right_motor'],
    motor_num['front_right_motor2'],
    motor_num['back_left_motor'],
    motor_num['back_left_motor2'],
    motor_num['back_right_motor'],
    motor_num['back_right_motor2'],
], dtype=int)

# モーターの条件を指定
amp = np.pi / 6
freq = 1.0
omega = 2*np.pi*freq

# ロボットの位置を見るためにボディを取得
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "board1")

# モーターを使ってロボットを動かす関数
def move_robot(data, t, phases):
	data.ctrl[motor_ids] = amp * np.sin(omega*t + phases)

# シミュレーション時間の変数
dt = 0.002
episode_time = 4.0
steps = int(episode_time/dt)

XML = "two_freedom_insect.xml"
BODY_NAME = "board1"

def evaluate(phases):
	# 評価する際は毎回初期状態に戻す必要があるから、モデルを初期化する
	model = mujoco.MjModel.from_xml_path(XML)
	data = mujoco.MjData(model)

	model.opt.gravity[:] = [0, 0, -9.81]

	start_x = data.xpos[body_id][0]

	# 施行を回す部分
	for step in range(steps):
		t = step * dt
		move_robot(data, t, phases)
		mujoco.mj_step(model, data)

	end_x = data.xpos[body_id][0]
	reward = float(end_x - start_x)
	return reward

# # 最適化ループ
# dimention = 8
# x0 = np.random.uniform(0, 2*np.pi, dimention)
# sigma0 = 0.5

# es = cma.CMAEvolutionStrategy(
#     x0,
#     sigma0,
#     {
#         'bounds': [0.0, 2*np.pi],
#         'popsize': 16
#     }
# )

# while not es.stop():
#     solutions = es.ask()
#     losses = []

#     for x in solutions:
#         r = evaluate(np.array(x))
#         losses.append(-r)   # CMA-ES は最小化

#     es.tell(solutions, losses)
#     es.disp()


# best_phase = np.mod(es.result.xbest, 2*np.pi)
# np.save("best_phase.npy", best_phase)
# print("saved best_phase.npy")

# ====== 可視化専用 ======
best_phase = np.load("best_phase.npy")

model_vis = mujoco.MjModel.from_xml_path("two_freedom_insect.xml")
data_vis = mujoco.MjData(model_vis)

model_vis.opt.gravity[:] = [0, 0, -9.81]

body_id = mujoco.mj_name2id(
    model_vis, mujoco.mjtObj.mjOBJ_BODY, "board1"
)

t0 = time.time()

with viewer.launch_passive(model_vis, data_vis,
                           show_left_ui=False,
                           show_right_ui=False) as v:
    while v.is_running():
        t = time.time() - t0
        move_robot(data_vis, t, best_phase)
        mujoco.mj_step(model_vis, data_vis)
        v.sync()
