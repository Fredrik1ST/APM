import time
import numpy as np

# Intialize parameters
j = 0           # Counter for updating CC parameters
ti_CC = 0       # Initial time for start of CC phase
Vi_CC = 0       # Initial velocity for start of CC pahse 
Vd_APM = 0      # Current desirec velocity
Vd_last = 0     # Last value of Vd

# Sigmoid parameters
k_a = 1.5       # Sigmoid steepness parameter for acceleration (m/s)
k_d = 1.5       # Sigmoid steepness parameter for deceleration (m/s)
s_a = 7         # Sigmoid shift parameter for acceleration
s_d = 7         # Sigmoid shift parameter for deceleration
expedite_t = 7  # Expedite time of the acceleration

def acceleration_phase(t_i, v_i, v_d, k, s):

    '''
    t_i: initial time
    v_i: inital velocity
    v_d: desired velocity
    k:   sigmoid steepness parameter
    s:   sigmoid shift parameter
    '''
    t = time.time()

    sigmoid = 1 / (1 + np.exp(-k * ((t - t_i) - s)))

    v_sigmoid = v_i + (v_d - v_i)*sigmoid

    return v_sigmoid

def updateCCparameters(Vd_list, p_segment_list, total_pos_travelled, speed_APM):
    
    global j, Vd_APM, Vi_CC, ti_CC, Vd_last 

    if j < len(Vd_list) and (total_pos_travelled >= p_segment_list[j] - Vd_list[j]*expedite_t):
        j+= 1
        Vd_last = Vd_APM
        Vd_APM = Vd_list[j]
        Vi_CC = speed_APM
        ti_CC = time.time()


def cruise_controller():
    
    if Vd_APM > Vi_CC:
        phase_var = 'acceleration'
    elif Vd_APM < Vi_CC:
        phase_var = 'deceleration'
    elif Vd_APM == 0:
        phase_var = 'constant_position'
    else:
        phase_var = 'constant_velocity'
    
    match phase_var:
        case 'acceleration': # First phase: acceleration
            cc_speed_ref = acceleration_phase(ti_CC, Vi_CC, Vd_APM, k_a, s_a)
        case 'constant_velocity': # Second phase: constant velocity
            cc_speed_ref = Vd_APM
        case 'deceleration': # Third phase: deceleration
            cc_speed_ref = acceleration_phase(ti_CC, Vi_CC, Vd_APM, k_d, s_d)
        case 'constant_position': # Fourth phase: constant positiion
            cc_speed_ref = 0
        case _:
            print('Error')
    if(cc_speed_ref < 0):
        cc_speed_ref = 0

    return cc_speed_ref

    
