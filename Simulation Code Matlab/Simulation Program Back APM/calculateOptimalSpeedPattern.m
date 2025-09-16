function optimalSpeedArray = calculateOptimalSpeedPattern(distanceList, speedList, distance, t_f)
    % Takes in the optimal speed pattern for a runner to keep in a given distance, 
    % and return the speed pattern a runner should keep to finish in a given
    % finish time
    %
    % Inputs;
    %   distanceList        = array of distance per speed                    (m)
    %   speedList           = array consisting of optimal speed pattern     (m/s)
    %   distance            = total distance of race/session                 (m)
    %   t_f                 = desired finishing time                         (s)
    %
    % Output_
    %   optimalSpeedArray   = shifted speed array per distance segment      (m/s)

    % Calculate average of optimal speed pattern
    v_avg_elite = mean(speedList);
    N = length(speedList);

    % Find average to finish in a given time
    v_avg_desired = (distance / t_f);
    %Move graph to keep to desired pace
    optimalSpeedArray = [];
    change_speed = abs(v_avg_desired - v_avg_elite);
    optimalSpeedArray(1) = 0;
    for i=2:N
        optimalSpeed = (speedList(i) - change_speed);
        optimalSpeedArray(i) = optimalSpeed;
    end
    
    %{
    figure
    hold on
    plot(x,y, '*-');
    hold on
    yline(v_avg, '--','Color', 'b');
    hold on
    plot(x,optimalSpeedArray, '*-');
    hold on
    yline(v_avg_desired, '--','Color', 'r');
    xlabel('Distance (m)');
    ylabel('Velocity (m/s)');
    legend('Professional');
    legend('Elite', 'v_{avg,e}', 'Athlete','v_{avg,a}');
    %}

end