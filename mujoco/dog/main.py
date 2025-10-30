import numpy as np
import time

import mujoco
import glfw

model = mujoco.MjModel.from_xml_path("./quadruped.xml")
data  = mujoco.MjData(model)

if not glfw.init():
    raise RuntimeError("Fail to GLFW intialization")

window = glfw.create_window(800, 600, "MuJoCo GLFW Viewer", None, None)
glfw.make_context_current(window)

scene = mujoco.MjvScene(model, maxgeom=1000)
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)

cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
cam.elevation = -20
cam.azimuth = 120
cam.trackbodyid = 0

t0 = time.time()
omega = 2*np.pi*1.5

while not glfw.window_should_close(window):
    t = time.time() - t0

    cam.azimuth = (omega*t)%360    # カメラの回転
    
    if t > 1: # 最初の1 sは静止
        data.ctrl[0]  = 1 #0*np.sin(omega*t + np.pi/2 * 1)
        data.ctrl[1]  = 1 #0*np.sin(omega*t + np.pi/2 * 1)
        data.ctrl[2]  = -1*np.sin(omega*t + np.pi/2 * 1)
        data.ctrl[3]  = 1*np.sin(omega*t + np.pi/2 * 1)        
        data.ctrl[4]  = -1*np.sin(omega*t + np.pi/2 * 1)
        data.ctrl[5]  = 1*np.sin(omega*t + np.pi/2 * 1)        

        data.ctrl[6]  = 1 #0*np.sin(omega*t + np.pi/2 * 3)
        data.ctrl[7]  = 1 #0*np.sin(omega*t + np.pi/2 * 3)
        data.ctrl[8]  = -1*np.sin(omega*t + np.pi/2 * 3)
        data.ctrl[9]  = 1*np.sin(omega*t + np.pi/2 * 3)        
        data.ctrl[10]  =-1*np.sin(omega*t + np.pi/2 * 3)
        data.ctrl[11]  = 1*np.sin(omega*t + np.pi/2 * 3)        

        data.ctrl[12]  = 1 #0*np.sin(omega*t + np.pi/2 * 0)
        data.ctrl[13]  = 1 #0*np.sin(omega*t + np.pi/2 * 0)
        data.ctrl[14]  = -1*np.sin(omega*t + np.pi/2 * 0)
        data.ctrl[15]  = 1*np.sin(omega*t + np.pi/2 * 0)        
        data.ctrl[16]  = -1*np.sin(omega*t + np.pi/2 * 0)
        data.ctrl[17]  = 1*np.sin(omega*t + np.pi/2 * 0)        

        data.ctrl[18]  = 1 #0*np.sin(omega*t + np.pi/2 * 2)
        data.ctrl[19]  = 1 #0*np.sin(omega*t + np.pi/2 * 2)
        data.ctrl[20]  = -1*np.sin(omega*t + np.pi/2 * 2)
        data.ctrl[21]  = 1*np.sin(omega*t + np.pi/2 * 2)        
        data.ctrl[22]  = -1*np.sin(omega*t + np.pi/2 * 2)
        data.ctrl[23]  = 1*np.sin(omega*t + np.pi/2 * 2)        

    mujoco.mj_step(model, data)
    
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, con)

    glfw.swap_buffers(window)
    glfw.poll_events()

    #print(t, data.ctrl[4])
    
glfw.terminate()

            
            
        

