function [v_current, e_int, e_prev, v_filter_last] = PID_and_DCmotor(Ts, v_current, v_ref, v_filter_last, alpha, e_gps, e_int, e_prev, Kp, Kd, Ki, max_int, min_int)
% This function calculates the current velocity for the APM using PI and a DC motor simulation. 
% It also incorporates a low-pass filter to handle measurement noise.
%
% Inputs:
%   Ts            = Sampling time                                   (s)
%   v_current     = Current velocity                                (m/s)
%   v_ref         = Desired velocity                                (m/s)
%   v_filter_last = Last filtered velocity                          (m/s)
%   alpha         = Filter parameter for handling measurement noise
%   e_gps         = GPS error to be incorporated into the filter    (m/s)
%   e_int         = Integral of the error                           (m/s)
%   e_prev        = Previous error                                  (m/s)
%   Kp, Kd, Ki    = PID controller gains
%   max_int       = Maximum value for the integral term             (m/s)
%   min_int       = Minimum value for the integral term             (m/s)
%
% Outputs:
%   v_current     = Updated current velocity                        (m/s)
%   e_int         = Updated integral of the error                   (m/s)
%   e_prev        = Updated previous error                          (m/s)
%   v_filter_last = Updated last filtered velocity                  (m/s)
%
% Author:    Camilla Kvamme
% Date:      2023 
%
% This code is copied from Camilla Kvamme's thesis 
% "Utvikling av en autonom hare for løp på bane"
    v_filter = (1 - alpha) * v_filter_last + alpha * (v_ref + e_gps);
    if(v_filter < 0.05)
        v_filter = 0;
    end

    v_filter_last = v_filter; 
    
    % Update PID error parameters
    e_p = v_filter - v_current; 
    %e_p = v_ref - v_current; 
    e_int = e_int + e_prev * Ts; 
    e_d = (e_p - e_prev)/Ts;     
    e_prev = e_p; 
    
    % Anti-windup (limit the integral term)
    e_int = max(min(e_int, max_int), min_int);

    % PID regulator
    u = Kp * e_p + Kd * e_d + Ki * e_int; 
    
    % Input voltage to the dc motor
    v_current = plant_DCmotor(Ts , [u u], v_ref);
    
    % Prevent speed to be negative
    if v_current <=0
        v_current = 0;
    end

end