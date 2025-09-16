function [lamp_bool, lamp_counter] = blinking_lamp(t, expedite_t, t_start, t_interval, t_segments, j, Ts, lamp_counter, p_error, interval_bool, lamp_bool, strategy_shifted);
% This is a function to control the blinking behavior of a lamp on the APM
%
% Inputs:
%   Ts           = Sampling time (s)
%   lamph_counter= Counter for controlling the lamp blinking
%   p_error      = Measured error
%
% Outputs:
%   lamph_bool   = Signal to the lamp 

 % Check if measured distance (d_current) is outside bounds, then send

    if interval_bool
            if (t > t_start - expedite_t) 
                if((t_segments(j) - t_interval) < 2) && (t_segments(j) > 0) && (t_segments(j) - t_interval > 0)
                    % Lamp starts blinking (0.1 sec on, 0.1 sec off)
                    if (lamp_counter >= 0.2)
                        lamp_bool = true;
                        lamp_counter = lamp_counter + Ts;
                        if (lamp_counter >= 0.4)
                            lamp_counter = 0;
                            lamp_bool = false;
                        end
                    else 
                        lamp_counter = lamp_counter + Ts;
                        lamp_bool = false;
                    end
                else
                    if strategy_shifted == 1
                        lamp_bool = false;
                    else
                        lamp_bool = true;
                    end
                    lamp_counter = 0;
                end
            end

    else
        if abs(p_error) >= 10   
            % Lamp start blinking (1 sec on, 1 sec off)
            if(lamp_counter >= 1)
                lamp_bool = true;
                lamp_counter = lamp_counter + Ts;
                if(lamp_counter >= 2)
                    lamp_counter = 0;
                    lamp_bool = false;
                end
            else
                lamp_counter = lamp_counter + Ts;
                lamp_bool = false;
            end
        else
            % Lamp off
            lamp_bool = false;
        end
    end
end