function [time, positions] = TrapezoidalPosition(distance_data, speed_data)


    % Preallocate time array with NaN to later fill with calculated values
    time = zeros(1, length(distance_data));

    % Initial time is zero
    time(1) = 0;

    % Apply the trapezoidal rule to calculate the time at each distance data point
    for i = 2:length(distance_data)
        % Calculate the average speed between the two data points
        avg_speed = (speed_data(i-1) + speed_data(i)) / 2;

        % Calculate the time it takes to travel from the previous distance point to the current one
        time(i) = time(i-1) + (distance_data(i) - distance_data(i-1)) / avg_speed;
    end

    % Interpolate to get the position at each time step
    positions = interp1(time, distance_data, 0:max(time), 'linear', 'extrap');
end
