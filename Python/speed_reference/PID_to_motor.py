from speed_reference.GB_Controller import GBController as gbc



#integral_error = 0

max_int = 70                # Maximum value for the integral term
min_int = - 70              # Minimum value for the integral term 
max_velocity = 100 * 10/36  # Max velcoity of APM, (km/h converted to m/s)


def input_to_motor(speed_APM, v_ref, motor_controller):
    '''
    Modulates the input to a motor based on the velocity control demand using a PID motor_controller
    encoded with anti-windup and a low-pass filter. Outputs a control signal between 0 - 100 
    depending on precentage of max velocity

    Parameters:
        speed_APM (float): Current velocity of the APM (m/s)
        v_ref (float): Desired velocity (setpoint) (m/s)
        Ts (float): Sampling time (s)

    Returns:
        input_motor: Motor input as a percentage of maximum velocity (0 - 100%)
        Gives out value between 0 - 100 depending on precentage of max velocity
    '''

    global v_filter_GPS_last, integral_error

    #error = v_ref - speed_APM

    # Anti-windup (limit the integral term)
    #integral_error = max(min(integral_error, max_int), min_int)

    # Update motor_controller t
    motor_controller.setpoint = v_ref

    # Compute control signal
    

    # Anti-windup (limit the integral term)
    motor_controller.integral = max(min(motor_controller.integral, max_int), min_int)
    #integral_error = motor_controller.integral  # update integral error

    u = motor_controller.compute(speed_APM)

    # Convert the change in velocity to a percentage of the maximum velocity
    input_to_motor = u

    return input_to_motor
