import time

start_time = None 

def start_timer():
    global start_time
    start_time = time.time()

def check_timer(preset_value):
    global start_time
    
    if start_time is None:
        return False
    
    elapsed_time = time.time() - start_time
    
    return elapsed_time >= preset_value


