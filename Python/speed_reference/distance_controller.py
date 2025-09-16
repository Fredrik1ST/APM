

def distance_controller(speed_APM, runner_pos, set_point, dc_pid):

    """
    Controls the speed change based on the distance to a runner and adjusts the set point.

    Parameters:
        speed_APM (float): Current velocity of the APM (m/s)
        runner_pos (float): Current position of the runner
        set_point (float): Desired distance to maintain from the runner

    Returns:
        float: The new reference velocity (v_ref) to maintain the distance
    """

    # Update setpoint and current measurement
    dc_pid.setpoint = set_point

    # Compute the required change in speed to maintain the desired distance
    speed_change = dc_pid.compute(runner_pos)

    # Calculate the new reference velocity
    v_ref = speed_APM + speed_change
    
    if v_ref < 0:
        v_ref = 0

    return v_ref
