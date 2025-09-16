function [strategy_shifted, outside_bonds, d_wanted, stable_bool, t_shift, min_bool, previous_error, t_interval, j, switch_intervall_bool] = switch_function(t, t_start,delta_p, d_min, d_max, outside_bonds, gamma, p_error, d_ref_init, stable_bool, min_bool, d_wanted, strategy_shifted, t_shift, Ts, previous_error, t_interval, j, interval_bool,t_segments, expedite_t, switch_intervall_bool )
% This function if for switching between CC and DC
%
% Inputs:
% delta_p         = Current distance error between APM and runner            (m)
% d_min           = Minimum allowed distance between APM and runner          (m)
% d_max           = Maximum allowed distance between APM and runner          (m)
% outside_bonds   = Bool indicating if the distance is outside bounds        (1 or 0)
% gamma           = Parameter to regulate teh efferct of the refrence signal (m/s^2)
% p_error         = Proportional distance error                              (m)
% d_ref_init      = Initial reference distance (middle of bounderies)        (m)
% stable_bool     = Bool indicating stability                                (1 or 0)
% min_bool        = Bool indicating if the runner runs to fast or slow       (1 or 0)
% d_wanted        = Desired distance for control adjustment                  (m)
% strategy_shifted= Bool indicating a strategy shift                         (1 or 0)
% t_shift         = Counter to know if the DC is stable                      (s)
% Ts              = Sampling time                                            (s)
% previous_error  = Previous distance error                                  (m)
%
% Outputs:
% strategy_shifted= Updated bool indicating a strategy shift                 (1 or 0)
% outside_bonds   = Updated bool indicating if the distance is outside bounds(1 or 0)
% d_wanted        = Updated desired distance for control adjustment          (m)
% stable_bool     = Updated bool indicating stability                        (1 or 0)
% t_shift         = Updated time of the last strategy shift                  (s)
% min_bool        = Updated bool indicating minimum distance violation       (1 or 0)
% previous_error  = Updated previous distance error                          (m)
%
%
% Author:    Camilla Kvamme and Veronica Kenworthy 
% Date:      13.03.2024

% If the runner is to close/farway from the APM and goes outside the boundaries
% then udate the wanted distance between APM and runner (d_wanted), 
% and update the system to say the APM is outside the boundaries and switch to distance control.  
    if (delta_p < d_min || delta_p > d_max) && ~outside_bonds  
            if(delta_p > d_max)
                d_wanted = d_max;
                min_bool = false;
            elseif (delta_p < d_min)
                d_wanted = d_min; 
                min_bool = true;
            end
            
            % Update parameters to signal outside bounds 
            strategy_shifted = 2; 
            outside_bonds = true;
            
            % Calculate distance error for DC
            previous_error = gamma*d_wanted - delta_p;

    % if the position error based on the input and the position of the APM
    % is less than a max distance error (10) and has been outside
    % bounaies.
    elseif (abs(p_error) <= 10) && outside_bonds 
        % if the DC is stable move APM to the middle of the bounderies for disitance between runner and APM  
        if(stable_bool)
                 if (delta_p > d_ref_init - 1 && delta_p < d_ref_init + 1)
                     if(~interval_bool)
                        strategy_shifted = 1;
                     end
                     outside_bonds = false;
                     t_shift = 0;
                     stable_bool = false;                   
                 else
                     d_wanted = d_ref_init;
                 end
        %  Check if the APM is inside +- 0.05 of the wanted distance between
        %  APM and runner, if it is start to count
        elseif ((min_bool && delta_p > d_min - 0.05 && delta_p < d_min + 0.01) || (~min_bool && delta_p > d_max - 0.01 && delta_p < d_max + 0.01))
                 % Update time
                 t_shift = t_shift + Ts;
                 if(t_shift >= 2)
                     stable_bool = true;
                 end
        else
                 stable_bool = false;
                 t_shift =0;
                 if min_bool
                     d_wanted = d_min; 
                 else
                     d_wanted = d_max; 
                 end
        end
    elseif (abs(p_error) >= 10)
         stable_bool = false;
         t_shift =0;
         if min_bool
             d_wanted = d_min; 
         else
             d_wanted = d_max; 
         end
    end

    % If the intervall is finished swith from CC to DC, if rest is 
    % finished switch from DC to CC  
    if(interval_bool)
        if(t > t_start - expedite_t &&  j < length(t_segments) + 1) 
                if (t_interval >= t_segments(j) && t_segments(j)> 0) 
                    if(strategy_shifted == 1)
                        strategy_shifted = 2;
                        t_interval = 0;
                        % Calculate distance error for DC
                        previous_error = gamma*d_wanted - delta_p;
                    elseif (strategy_shifted == 2)
                        strategy_shifted = 1;
                        t_interval = 0;
                    end
                    switch_intervall_bool = true;
                else
                        t_interval = t_interval + Ts;
                end
        end
    end

end