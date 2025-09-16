######################################################################
# The code for even or odd numers is from: toppr
# Accsessed [24.04.2024]
#url: https://www.toppr.com/guides/python-guide/examples/python-examples/python-program-to-check-if-a-number-is-odd-or-even/
######################################################################

import controller_timer as timer
import decide_controller 

bool_light_green = False
bool_light_red = False
ti_cc = 0 # Initial time for the sigmoid func
first_time_outside_light =True
count_light = 0

def lampBlinking(timeBlink, bool_lamp):

    # Start timer if not yet started
    if timer.start_time is None:
        timer.start_timer()

    elif timer.check_timer(timeBlink): # Blink every 0.5 sec
        if(bool_lamp):
            bool_lamp = False
        else:
            bool_lamp = True
        timer.start_time = None
    return bool_lamp

def lampStartup(bool_readyToStart): #remeber to turn off light after the runner strarts to run
    global bool_light_green, bool_light_red

    if(bool_readyToStart):
        bool_light_red= False
        bool_light_green = True
    else:
        bool_light_red= True
    return bool_light_red, bool_light_green

def lampEnd():
    global bool_light_red, bool_light_green
    bool_light_red = True
    bool_light_green = False
    return bool_light_red

def lampDuringRun(bool_outsidebounds, p_error_limit, p_error_constant_lim):
    global first_time_outside_light, count_light
    # Blink tre times if it is the fist time its outside the bonds outside bonds
    if(bool_outsidebounds and first_time_outside_light):
        if(count_light <= 3):
            bool_light_green = lampBlinking(0.5, bool_light_green)
            count_light += 1
        else:
            first_time_outside_light = False
            count_light = 0

    # If the distance error (according to schedule) is larger than the maximum value turn on light, let it stay on until the runner is on schedule again
    if( p_error_limit >= p_error_constant_lim):
        bool_light_green = True
    else:
        bool_light_green  = False


def lampIntervall(t_segment, timeRunned, counter_interval_input): # Input the list of time segments from user input, and the time since the run started
    # counter_interval_input is the input saing with placement in the list you are at
    global bool_light_green , bool_rest
    if(timeRunned >= t_segment - 3): #Start blinking 3 seconds before end of interval
        bool_light_green = lampBlinking(0.5, bool_light_green)

    if(counter_interval_input % 2 == 0): # if its a even number the runner is at rest time. 
         bool_light_green  = True
    else: # if its a even number the runner is at running time.
         bool_light_green  = False
    
    return bool_light_green 