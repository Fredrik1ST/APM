function  [Vd_APM, Vi_APM, switch_var, ti, j, alpha, v_ref, Vd_last_APM,t_interval, switch_intervall_bool] = cruisecontroller(t, t_start, expedite_t, expedite_p, d_segments, Vd_tab, p_APM, Vd_APM, Vi_APM, v_APM, Vd_last_APM, switch_var, ti, j, alpha, v_ref, k_a, k_d, s_a, s_d, t_interval,interval_bool, t_segments, switch_intervall_bool)
% Returns refrence velocity
%
% Inputs:
% t             = Time                                              (s)
% t_start       = Time APM useses to start up                       (s)
% expedite_t    = Time constant for expediting reference changes    (s)
% expedite_p    = Constant calculated from the expediting           (m)
% d_segments    = Array of input distance segments                  (s)
% Vd_tab        = Array of desired velocities for each segment      (m/s)
% p_APM         = Current position                                  (m)
% Vd_APM        = Desired velocity for the current segment          (m/s)
% Vi_APM        = Initial velocity before accelerating              (m/s)
% v_APM         = Current APM velocity                              (m/s)
% Vd_last_APM   = Last desired velocity                             (m/s)
% switch_var    = Switch parameter for diffent phases               (-)
% ti            = Initial time of the current segment               (s)
% j             = Index of the current segment in d_segments        (-)
% alpha         = Parameter for lowpassfilter                       (-)
% v_ref         = Reference velocity                                (m/s)
% k_a           = Sigmoid steepness parameter for acceleration      (1/s)
% k_d           = Sigmoid steepness parameter for deceleration      (1/s)
% s_a           = Sigmoid shift parameter for acceleration          (-)
% s_d           = Sigmoid shift parameter for deceleration          (-)
%
% Outputs:
% Vd_APM        = Updated desired velocity for the current segment  (m/s)
% Vi_APM        = Updated initial velocity                          (m/s)
% switch_var    = Updated switch parameter                          (-)
% ti            = Updated start time of the current segment         (s)
% j             = Updated index of the current segment in d_segments(-)
% alpha         = Updated acceleration or deceleration parameter    (-)
% v_ref         = Reference velocity                                (m/s)
% Vd_last_APM   = Last desired velocity                             (m/s)
%
% Author:    Camilla Kvamme
% Date:      13.03.2024
%

% Start CC after the startup time 

    if (t > t_start - expedite_t &&  j ~= length(Vd_tab) + 1) 
            % If the position is larger than the set sqedule 
            % then update wanted speed (Vd_APM) for the APM.
            if ~interval_bool && (p_APM >= d_segments(j) - expedite_p(j)) 
                Vd_last_APM = Vd_APM;
                if(j < length(Vd_tab))
                    Vd_APM = Vd_tab(j+1);
                end
                Vi_APM = v_APM; % Set new initial velocity
                
                % Based on the new wanted speed (Vd_APM) 
                % decide witch phase the APM is in.
                if (Vd_APM > Vd_last_APM)
                    switch_var = 1;         % Acceleration phase
                    ti = t;                 % Update time initial time
                elseif (Vd_APM == 0)
                    switch_var = 4;         % Constant position phase
                elseif (Vd_APM < Vd_last_APM)
                    switch_var = 3;         % Deceleration phase
                    ti = t;                 % Update time initial time     
                elseif (Vd_APM <= Vi_APM + 0.001) && (Vd_APM >= Vi_APM - 0.001)
                    switch_var = 2;         % Constant velocity phase
                end
                % Update j 
                j = j + 1; 
                % Updating alpha for lowpass 
                alpha = 1;

            elseif interval_bool && switch_intervall_bool
                Vd_last_APM = Vd_APM;
                if(j < length(Vd_tab))
                    Vd_APM = Vd_tab(j+1);
                end
                Vi_APM = v_APM; % Set new initial velocity
                
                % Based on the new wanted speed (Vd_APM) 
                % decide witch phase the APM is in.
                if (Vd_APM > Vd_last_APM)
                    switch_var = 1;         % Acceleration phase
                    ti = t;                 % Update time initial time
                elseif (Vd_APM == 0)
                    switch_var = 4;         % Constant position phase
                elseif (Vd_APM < Vd_last_APM)
                    switch_var = 3;         % Deceleration phase
                    ti = t;                 % Update time initial time     
                elseif (Vd_APM <= Vi_APM + 0.001) && (Vd_APM >= Vi_APM - 0.001)
                    switch_var = 2;         % Constant velocity phase
                end

                % Update counter for the list of Vd_APM and time 
                j = j + 1; 
                switch_intervall_bool = false;

                % Updating alpha for lowpass 
                alpha = 1;
            end           
    end
    % Different phases in CC
    switch switch_var
        case 1 % First phase: acceleration
            [v_ref] = acceleration_phase( t, ti, Vi_APM, Vd_APM, k_a, s_a);
        case 2
           % Second phase: constant velocity
            v_ref = Vd_APM;
        case 3 % Third phase: deceleration  
            [v_ref] = acceleration_phase( t, ti, Vi_APM, Vd_APM, k_d, s_d); 

        case 4 % Forth phase: constant position    
            v_ref = 0;
    
        otherwise
            % Handle any other cases if needed
            display('Error');
    end
end