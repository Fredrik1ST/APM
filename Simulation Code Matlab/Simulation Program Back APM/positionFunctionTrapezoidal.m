function [positions_array, time_array] = positionFunctionTrapezoidal(t_f, distance_data, speed_data, Ts)
% Returns the position of the runner at time t
%
% Outputs:
%   positions_array     = array of positions the runner have at each time step
%   time_array          = array of each time step
%
% Inputs:
%   distanceList        = array of distance per speed
%   speedList           = array consisting of optimal speed pattern
%   distance            = total distance of race/session
%   t_f                 = desired finishing time
%   Ts                  = time step for each position
%
% Output:
%   optimalSpeedArray   = shifted speed array per distance segment

    % Initialize variables
    % Determine the number of time steps based on the total desired time and time step
    num_time_steps = floor(t_f / Ts);

    % Initialize arrays to hold position and time at each time step
    positions_array = zeros(1, num_time_steps);
    time_array = Ts * (0 : num_time_steps - 1); % Time array from 0 to t_f in steps of Ts

    pos = 0; % Initial position
    i = 1;  % Index to keep track of the current interval in speed data

   
    % Loop over the time steps until the input time is reached
    for t = 1: num_time_steps
        % Check if we have reached the end of the distance data
        if  pos >= distance_data(end)
            positions_array(t:end) = distance_data(end);
            break;
        end

        % Get the starting and ending speed for the current interval
        v_start = speed_data(i);
        if i < length(speed_data)
            v_end = speed_data(i + 1);
        else
            v_end = speed_data(i);
        end

        % Calculate the average speed for the current interval using the trapezoidal rule
        v_avg = (v_start + v_end) / 2;
        
        % Update the position using the trapezoidal rule
        pos = pos + v_avg * Ts;
        positions_array(t) = pos;

        % Move to the next interval if the end of the current interval is surpassed
        if i < length(distance_data) && pos >= distance_data(i)
            i = i + 1; % Increment the index to the next interval
        end
        
        time_array(t) = t + Ts;

    end
    
end






