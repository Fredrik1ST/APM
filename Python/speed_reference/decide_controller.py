import speed_reference.controller_timer as timer
#import lights_logic

control_strategy = 'cruise_control'
d_ref = 0
stable_within_bounds = True
within_bounds = True
pos_error_limit = 20 
pos_error = 0
d_pos_travelled = 0

def updatePosError(total_pos_travelled, Vd, d_init):
        
        global pos_error, d_pos_travelled

        #Start timer if not yet started
        if timer.start_time is None:
            timer.start_timer()
        
        elif timer.check_timer(1):      
            d_pos_travelled += Vd
            pos_error = total_pos_travelled - (d_pos_travelled - d_init)
            timer.start_time = None


def check_if_stable_within_bounds(runner_pos):
    
    t_shift = 2
    delta = 0.5 # Allowed deviation from desired setpoint d_ref
    global stable_within_bounds

    if (runner_pos > d_ref - delta) or (runner_pos < d_ref - delta):

        #Start timer if not yet started
        if timer.start_time is None:
            timer.start_timer()
        
        elif timer.check_timer(t_shift):
            stable_within_bounds = True
            timer.start_time = None

    return stable_within_bounds

def check_within_bounds(runner_pos, d_min, d_max, d_init):

    global d_ref
    global stable_within_bounds
    global within_bounds

    if runner_pos > d_min and runner_pos < d_max:
        within_bounds = True
    else:
        within_bounds = False
        stable_within_bounds = False

        if runner_pos < d_min:
            d_ref = d_min
        elif runner_pos > d_max:
            d_ref = d_max

    if pos_error < pos_error_limit:
        stable_within_bounds = check_if_stable_within_bounds(runner_pos)

    if stable_within_bounds:
        d_ref = d_init

def decide_control_method(runner_pos, d_init):
    '''
    Decide if the system should be in Cruise Control or Distance Control
    '''
    # Variables to be changed
    global control_strategy 

    delta = 1 # Allowed deviation (m)
   
    if (within_bounds and stable_within_bounds):
        if (d_init - delta < runner_pos < d_init + delta):
            # Shift to CC if stable_within_bounds and the distance to runner is satisfactory close to d_init
            control_strategy = 'cruise_control'
            #lights_logic.first_time_outside_light = True

    elif not within_bounds:  
        control_strategy = 'distance_control'
