function [v_runner, last_var,v_runner_filter_last] = simulation_runner(t, i, v_runner, p_ref, max_acceleration, V_tab_runner, last_var, Vd_pulse, v_APM, v_runner_filter_last)
    % Simulating the runner's velocity


    % Simulating the runner's velocity 5000m
    if p_ref <= 4.5
        v_runner = 0;

    else
        if (i >= length(V_tab_runner))
            v_runner = v_runner;
        else
            if (t >= 40 && t <= 60)
                    last_var = last_var + (rand() - 0.008) * 2 * 0.008;
                    v_runner = V_tab_runner(i+1) + last_var;
            elseif (t >= 60 && t <= 150)
                    v_runner = v_runner +  (rand() - 0.00001) * 2 * 0.00001;
            elseif (t >= 120 && t <= 125)
                    v_runner = v_runner -  (rand() - 0.01) * 2 * 0.01;
            elseif (t >= 300 && t <= 320)
                    v_runner = v_runner - (rand() - 0.008) * 2 * 0.008;
            elseif (t >= 320 && t <= 405)
                     v_runner = v_runner;
            elseif (t >= 405 && t <= 415)
                    v_runner = v_runner + (rand() - 0.005) * 2 * 0.005;
                    last_var = 0;
            
            elseif V_tab_runner(i+1) == 0
                 v_runner = 0;
            else
                v_runner = V_tab_runner(i+1);
            end
        end
    end

   % Simulating the runner's velocity 1500
   % if (i >= length(V_tab_runner))
   %          v_runner = v_runner;
   % elseif (t <= 17)
   %          v_runner = V_tab_runner(i+1) + last_var - (rand() - 0.05) * 2 * 0.05;
   % else
   %      v_runner = V_tab_runner(i+1) + last_var - (rand() - 0.1) * 2 * 0.1;
   % end

    % Simulating the runner's velocity 1500
    % if p_ref <= 4.5
    %     v_runner = 0;
    % else
    %     if (i >= length(V_tab_runner))
    %         v_runner = v_runner;
    %     else
    %         if (t >= 17 && t <= 23)
    %                 last_var = last_var + (rand() - 0.01) * 2 * 0.01;
    %                 v_runner = V_tab_runner(i+1) + last_var+ 0.08;
    %         elseif (t >= 23 && t <= 30)
    %                  v_runner = v_runner;
    %         elseif (t >= 30 && t <= 33)
    %                 v_runner = v_runner - (rand() - 0.05) * 2 * 0.05;
    %                 last_var = 0;
    %         elseif (t >= 104 && t <= 110)
    %                 last_var = last_var + (rand() - 0.02) * 2 * 0.02;
    %                 v_runner = V_tab_runner(i+1) - last_var;
    %         elseif (t >= 110 && t <= 115)
    %                  v_runner = v_runner;
    %         elseif (t >= 115 && t <= 117)
    %                 v_runner = v_runner + (rand() - 0.05) * 2 * 0.05;
    %         elseif V_tab_runner(i+1) == 0
    %             v_runner = 0;
    %         else
    % 
    %             v_runner = V_tab_runner(i+1);
    %         end
    %     end
    % end


% In this simulation runner and APM has the exact speed
    % if(i >= length(V_tab_runner))
    %     v_runner = v_runner;
    % else
    %     v_runner = V_tab_runner(i+1);
    % end
end