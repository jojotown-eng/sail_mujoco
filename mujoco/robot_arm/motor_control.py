import numpy as np
import time

def VelocityControl(now_vel, goal_vel):
	now_error = goal_vel - now_vel
	output_value = 0.5*now_error + pre_error*0.01
	pre_error = now_error
	return output_value

def PositionControl(target, now_position, kp, ki, kd):
    if(error_total > 2.0):
         error_total = 2.0
    error = target - now_position
    control_value = error * kp + ki * error_total + kd * pre_error
    pre_error = error
    error_total+=error
    return control_value


def lerp(a, b, t):
    return a + (b - a) * t