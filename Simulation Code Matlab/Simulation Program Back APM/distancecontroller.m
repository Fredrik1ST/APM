function [Vd_APM, j, v_ref, previous_error, integral_error, alpha, switch_intervall_bool] = distancecontroller(p_ref, d_segments, expedite_p, expedite_t, Vd_tab, j, v_runner, Vd_APM, gamma, delta_p, Ts, Kp_DC, Ki_DC, Kd_DC, beta, d_wanted, v_APM, previous_error, integral_error, v_ref_last, t_interval, t_segments,interval_bool, switch_intervall_bool)
% Calculates the desired velocity for the APM, based on distance between runner and APM.
%
% Inputs:
% p_ref             = Reference position                             (m)
% d_segments        = Array of segment end positions                 (m)
% expedite_p        = Constant calculated from the expediting        (m)
% Vd_tab            = Array of desired velocities for each segment   (m/s)
% j                 = Index of the current segment in d_segments     (-)
% v_runner          = Velocity runner                                (m/s)
% Vd_APM            = Desired velocity for the APM based on segments (m/s)
% gamma             = Gamma parameter                                (-)
% delta_p           = Current distance between runner and APM        (m)
% Ts                = Sampling time                                  (s)
% Kp_DC             = Proportional gain for distance control         (-)
% Ki_DC             = Integral gain for distance control             (-)
% Kd_DC             = Derivative gain for distance control           (-)
% beta              = Beta parameter                                 (-)
% d_wanted          = Desired distance between APM and runner        (m)
% v_APM             = APM velocity                                   (m/s)
% previous_error    = Previous error                                 (m)
% integral_error    = Integral error                                 (-)
% v_ref_last        = Previous reference velocity                    (m/s)
%
% Outputs:
% Vd_APM            = Updated desired velocity for the APM           (m/s)
% j                 = Updated index of the current segment           (-)
% v_ref             = Reference velocity for the APM                 (m/s)
% previous_error    = Updated previous error in position             (m)
% integral_error    = Updated integral error                         (-)
% alpha             = Parameter for Low-pass filter                  (-)   
%
% Author:    Veronica Kenworthy, Enya..., Camilla Kvamme
% Date:      13.03.2024
%
% Some of this code is inspired from the thesis titled "", by Veronica Kenworthy and Enya


% If the position is larger than the set schedule
    % then update wanted speed (Vd_APM)
% If the position is larger than the set sqedule 
    % then update wanted speed (Vd_APM) for the APM. Mostly done for
    % plotting
    if ~interval_bool && (j < length(Vd_tab) && p_ref >= d_segments(j) - expedite_p(j))
         Vd_APM = Vd_tab(j+1);
          % Update counter
          j = j + 1; 
    elseif interval_bool && switch_intervall_bool
          Vd_APM = Vd_tab(j+1);
          % Update variables
          j = j + 1; 
          switch_intervall_bool = false;
    else 
        v_runner = v_runner;
        Vd_APM = Vd_APM;
    end


    % PID Control for calculating speed to make up for deviation in
    % distance.
    % beta-gamma-PID
    % derivative_error =  (previous_error - (gamma*d_wanted - delta_p)) / Ts;
    % integral_error = integral_error + (d_wanted - delta_p) * Ts;
    % u_DC = Kp_DC * (beta*d_wanted - (delta_p)) + Ki_DC * integral_error + Kd_DC * derivative_error; % Using filtered value in PID  
    % 
    % v_ref = v_APM + u_DC;   
    % 
    % previous_error = gamma*d_wanted - delta_p;

   % PID Control for calculating speed to make up for deviation in
    % distance.
    integral_error = integral_error + (d_wanted - delta_p) * Ts;
    derivative_error = ((gamma*d_wanted - delta_p) - previous_error) / Ts;
    v_ref = v_APM + Kp_DC * (beta*d_wanted - delta_p) + Ki_DC * integral_error + Kd_DC * derivative_error;

    previous_error = (gamma*d_wanted - delta_p);

    % Keep the distance from being negative
    if p_ref <= 4.5
        p_ref = 4.5;
    end
    v_ref_last = v_ref;

    % Updating t in intervall
    t_interval = t_interval + Ts;
    
    % Updating alpha for lowpass 
    alpha = 1;
end