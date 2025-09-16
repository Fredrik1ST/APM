%% Simulation program for the refeence model, and speed regulation of the APM 

% Author:    Camilla Kvamme and Veronica Kenworthy
% Date:      15.03.2024

% ChatGPT has been used to help with commenting the headlines of the
% functions in this program

% The code for the function  "PID_and_DC" is copied from Camilla Kvamme's 
% thesis "Utvikling av en autonom hare for løp på bane"
% The "Distance Control" function is from the thesis by Veronica 
% and Enya titled "Development of an Autonomous Rabbit for Track Running."
% but has been altred in this theis.

clear;
clc;
close all;
%load('test1_800_runner.mat');
%load('test3_1500_runner.mat');
%load('test4_5000_runner.mat');
%load('test5_10000_runner.mat');
%load('test6_interval_runner.mat');
% Final tests
%load('test7_finalTest1500.mat');
load('test8_finalTest5000.mat');
%% Input by user
% Open and import data from Exelfile of the elite runner
interval_bool = false; % Valg av bruker

   if(interval_bool)
    % User inputs the segments of time they want to run an intervall
    % and the Speed they want to achive in the intervall
    filename = 'pace_interval.xlsx'; 
    [Vd_tab, t_segments] = getdata(filename); % ([m/s, m])
    d_segments = 0;
    % Calculate d_segments
      
   else
    % User inputs time they whant to finish on,sistance they want to run
    % and witch sex they want to follow.
    tf_input = 130; % Desired finish time inputed by user(s)
    distance_input = 5000; % Desired running distance inputed by user(s)
    sex_input = 'female'; % Decide between male and female elite runner
    t_segments = 0;

    % Calculate optimal speed based on elite runners
    [Vd_tab, d_segments] = decide_eliteStrategy(sex_input, distance_input, interval_bool);
    %løperen er ikke simulert med denne, men den hører til
    %Vd_tab = calculateOptimalSpeedPattern(d_segments, Vd_tab, distance_input, tf_input);
   end

%% Initialization of simulation parameters
Ts = 1/ 10;                                 % Sampling Time (s)
ti = 0;                                     % Initial time (s)
t = 0;                                      % Time (s)
tf = Vd_tab(end)*d_segments(end);           % Finish time run (s)
t_start = 10;                               % Time before APM starts (In simulated interval use t_start = 10)
Vi_APM = 0;                                 % Initial velocity for APM (m/s)
Vd_APM = 0;                                 % Desired velocity for APM (m/s)
Vd_last_APM = 0;                            % Previous desired velocity for APM (m/s)
v_APM = 0;                                  % Current speed for APM (m/s) 
v_ref_last = 0;                             % Last refrence speed from DC and CC
first_time = true;                          % Boolian for the first time inside if
strategy_shifted = 1;                       % Switch between CC an DC
switch_var = 4;                             % Switch parameter for CC
epsilon = 0.1;                              % Small velocity threshold for stopping the APM if runner is stopping
v_ref = 0;                                  % Reference speed calculated in CC and DC (m/s)
j = 1;                                      % Counter to update the desired speed (Vd_APM)
lamp_counter = 0;                          % Constant for time lamp is on 
lamp_bool = false;


%% Tunable parameters for distance control 
d_max = 8;                                  % Max distance between runner and APM
d_min = 4;                                  % Min distance between runner and APM
d_wanted = (d_max + d_min) / 2;             % Wanted distance between runner and APM
% 
% Kp_DC = 0.0192;
% Ki_DC = 0;
% Kd_DC = 0.0002131;

% Kp_DC = 0.1; % Proportional gain
% Ki_DC = 0.0; % Integral gain
% Kd_DC = 0.05; % Derivative gain

Kp_DC = 0.3;
Ki_DC = 0;
Kd_DC = 0.06;


% Used in Simulation test
% Kp_DC = 1.8;                                % Proportional gain for DC
% Ki_DC = 0.0;                                % Integral gain for DC
% Kd_DC = 0.05;                               % Derivative gain for DC

gamma = 0.1;
beta = 1;

% Initialization of parameters based on the DC
p_APM = (d_max + d_min) / 2;                % Refrence position for the APM (m)
p_ref= p_APM;                               % Refrence position for the APM (m)
p_error = 0;                                % Distance error based on the input sqeule and the APM (m)
integral_error = 0;                         % Integral error for DC
previous_error = 0;                         % Previous error for derivative calculation for DC
error_wanted_distance = 0;                  % Error between the wanted distance and the current distance for PD in DC 
t_shift = 0;                                % Counter when the distance between runner and APM is outside bounderies
stable_bool = false;                        % Bool to signal stable
min_bool = false;                           % Bool to say if the runner runs to fast/slow

% Variables to simulate how the system would react if it was following a
% step function (not follow a CC, but a step)
k = 1;                                      % Counter for simulation based on the step input
p_pulse = (d_max + d_min)/2;                % Position for testing, based on the step function 
Vd_pulse = 0;                               % Velocity for testing, based on the step function

%% DC motor 
t = 0.1; % Time constant
j = 1; % Counter that updates the time from dat
outside_bonds = false;

%% Tuning parameters curise control
% Difference in Sigmod graph based on short distance and long distance 
k_a = 1.5;                % Sigmoid steepness parameter for acceleration (1/s)
k_d = 1.5;                % Sigmoid steepness parameter for deceleration (1/s)
s_a = 7;                  % Sigmoid shift parameter for acceleration
s_d = 7;                  % Sigmoid shift parameter for deceleration
expedite_t = 7;           % Expedite time of the acceleration
%% Initialization of time distance control
t_outside_bounds = 0;         % Time spent outside distance boundaries (s)

%% PID for bottom controller (BC) to DC motor
Kp_BC = 80;                    % Propotional gain
Ki_BC = 1251;                    % Integral gain
Kd_BC = 0;                  % Derivate gain

% Used in simulation tests
% Kp = 15;                    % Propotional gain
% Ki = 45;                    % Integral gain
% Kd = 0.08;                  % Derivate gain

% Anti-windup limits 
max_int_BC = 70;              % Maximum value for the integral term
min_int_BC = -70;             % Minimum value for the integral term 

% Initialization GPS-terms
e_gps = 0;                    % GPS Error
e_int = 0;                    % Integral term
e_prev = 0;                   % Previous error for the derivative term

% Initialization filter
v_filter_last = 0;            % Last filtered velocity (m/s)
alpha = 1;                    % Filter value

%% Pre-calculations
 if(~interval_bool)  
    for i = 1:length(d_segments)   
        expedite_p(i) = Vd_tab(i)*expedite_t;  % Expedite position of the acceleration 
    end
 else
     expedite_p = 0;
 end
%% Runner simulation
% Runner initialization
v_runner = 0;                               % Current speed runner (m/s)
p_runner = 0;                               % Current runner position (m)

% Load the speed from APM
%V_tab_runner = simdata(:,7);                % Loaded from earlier simdata (m/s) 
V_tab_runner = simdata(:,2);                % Loaded from earlier simdata (m/s) 

max_acceleration = 0.1;                     % Max acceleration for runner (m/Ts)
d_ref_init = (d_max + d_min) / 2;
last_var_runner = 0;                        % Last variable in runner simulation   
v_runner_filter_last =0;                    % Last filtered value for runner simulation

%% interval traning
if (interval_bool)
    k_a = 3.5;                  % Sigmoid steepness parameter for acceleration (1/s)
    k_d = 1.5;                  % Sigmoid steepness parameter for deceleration (1/s)
    s_a = 3;                  % Sigmoid shift parameter for acceleration
    s_d = 5;                  % Sigmoid shift parameter for deceleration
    expedite_t = 3;           % Expedite time of the acceleration

    Kp_DC = 2;                                  % Proportional gain for DC
    Ki_DC = 0.01;                                % Integral gain for DC
    Kd_DC = 0.1;                                % Derivative gain for DC
end

t_interval = 0;
t_pulse = 0;
switch_intervall_bool = true;

%% For-loop
N = (tf+t_start)/(Ts); %sek
simdata = zeros(length(N), 11);  % Table of simulation without PID

for i = 1:N+1
    % Break the loop if the index 'j' exceeds the length of the table
    if~interval_bool && p_APM >= d_segments(end) 
         break;
    elseif interval_bool && j > length(t_segments) 
         break;
    end

    if v_runner < epsilon
        % Stop the APM if the runner's velocity is very small
        v_APM = 0;
    end

    % Calculate distance between runner and APM
    delta_p = p_APM - p_runner;

    % Update The error between the runners distance and the wanted sqedule 
    if(~interval_bool)
        p_error = p_runner - (p_pulse - d_ref_init); 
    else
        % Do not check for distance error in intervall training. Not
        % nessesary since the pace in the rest is up to the user.
        p_error = 0;
    end


    % Simulation runner
    [v_runner, last_var_runner, v_runner_filter_last] = simulation_runner(t, i, v_runner, p_ref, max_acceleration, V_tab_runner, last_var_runner, Vd_pulse, v_APM, v_runner_filter_last);

    % Lamp function 
    [lamp_bool, lamp_counter] = blinking_lamp(t, expedite_t, t_start, t_interval, t_segments, j, Ts, lamp_counter, p_error, interval_bool, lamp_bool, strategy_shifted);
 
    % Switching strategy
    [strategy_shifted, outside_bonds, d_wanted, stable_bool, t_shift, min_bool, previous_error, t_interval, j, switch_intervall_bool] = switch_function(t, t_start,delta_p, d_min, d_max, outside_bonds, gamma, p_error, d_ref_init, stable_bool, min_bool, d_wanted, strategy_shifted, t_shift, Ts, previous_error, t_interval, j, interval_bool,t_segments, expedite_t, switch_intervall_bool);


    % Case for switching between CC and DC
     switch strategy_shifted 
        case 1 % Cruise Control
            display('C Control'); 
                % Cruise controller function
                [Vd_APM, Vi_APM, switch_var, ti, j, alpha, v_ref, Vd_last_APM,t_interval, switch_intervall_bool] = cruisecontroller(t, t_start, expedite_t, expedite_p, d_segments, Vd_tab, p_APM, Vd_APM, Vi_APM, v_APM, Vd_last_APM, switch_var, ti, j, alpha, v_ref, k_a, k_d, s_a, s_d, t_interval,interval_bool, t_segments, switch_intervall_bool);

        case 2 % Distance control
                display('D Control');
                % Distance controller function
                [Vd_APM, j, v_ref, previous_error, integral_error, alpha, switch_intervall_bool] = distancecontroller(p_ref, d_segments, expedite_p, expedite_t, Vd_tab, j, v_runner, Vd_APM, gamma, delta_p, Ts, Kp_DC, Ki_DC, Kd_DC, beta, d_wanted, v_APM, previous_error, integral_error, v_ref_last, t_interval, t_segments,interval_bool, switch_intervall_bool);
     end


     % Update the Vd_pulse based on the step input
     if(t > t_start && k ~= length(Vd_tab) + 1)
        % Find the desired velocity when the input and speed follow a impulse 
        if (~interval_bool) && (p_pulse >= d_segments(k))   
            if(k < length(Vd_tab))
                Vd_pulse = Vd_tab(k+1);
            end
            k = k+1;
        elseif (interval_bool) && (t_pulse > t_segments(k))
            if(k < length(Vd_tab))
                Vd_pulse = Vd_tab(k+1);
            end
            k = k+1;
            t_pulse=0;
        else
            t_pulse = t_pulse +Ts;
        end
     end
    
    % Lowpass, PID and DC motor (From Camilla's Thesis)
    [v_APM, e_int, e_prev, v_filter_last] = PID_and_DCmotor(Ts, v_APM, v_ref, v_filter_last, alpha, e_gps, e_int, e_prev, Kp_BC, Kd_BC, Ki_BC, max_int_BC, min_int_BC);

    % Update position 
    p_ref = p_ref + v_ref * Ts;
    p_APM =  p_APM + v_APM * Ts;
    p_pulse = p_pulse + Vd_pulse*Ts; 
    p_runner = p_runner + v_runner*Ts;
    %------------------------------------------------------------------

    % Store simulation data in a table (for plotting)
    simdata(i,:) = [t, v_ref, p_ref, Vd_pulse, p_pulse, v_APM, p_APM, delta_p, v_runner, p_error,lamp_bool];

    % Update time
    t = t + Ts;


end % End of for_loop


%% Plot
time          = simdata(:,1);                % min
vel_ref       = simdata(:,2);                % m/s
pos_ref       = simdata(:,3);                % m
vel_d_shift   = simdata(:,4);                % m/s
pos_shift     = simdata(:,5);                % m
vel_APM       = simdata(:,6);                % m/s
pos_APM       = simdata(:,7);                % m/s
d_p           = simdata(:,8);                % m
vel_run       = simdata(:,9);                % m/s
p_e           = simdata(:,10);               % m
lamp          = simdata(:,11);               % -

% Plot results
figure;
subplot(2, 1, 1);
plot(time, pos_ref, 'b', 'LineWidth', 1.5);
hold on;
plot(time, pos_shift, 'r--', 'LineWidth', 1.5); 
hold off;
xlabel('Time (sec)','FontSize', 16);
ylabel('Position (m)','FontSize', 16);
title('Position Reference','FontSize', 18);
legend('Position  Reference', 'Desired Position ', 'Location', 'Best','FontSize', 14);

subplot(2, 1, 2);
plot(time, vel_ref, 'b', 'LineWidth', 1.5); 
hold on;
plot(time, vel_d_shift, 'r--', 'LineWidth', 1.5); 
hold off;
xlabel('Time (sec)','FontSize', 16);
ylabel('Velocity (m/s)','FontSize', 16);
title('Velocity Reference','FontSize', 18);
legend('Velocity Reference', 'Desired Velocity (Vd)', 'Location', 'Best','FontSize', 14);
ylim([min(vel_ref), max(vel_ref) + 1]);

% subplot(3, 1, 3);
% plot(time, acc_ref, 'b', 'LineWidth', 1.5);
% xlabel('Time (min)');
% ylabel('Acceleration (m/s^2)');
% title('Acceleration Reference');
% 
figure;
subplot(2, 1, 1);
plot(time, pos_APM, 'b', 'LineWidth', 1.5);
hold on;
plot(time, pos_ref, 'r--', 'LineWidth', 1.5); 
hold off;
xlabel('Time (sec)','FontSize', 16);
ylabel('Position (m)','FontSize', 16);
title('Position APM','FontSize', 18);
legend('Position APM', 'Refrence Position', 'Location', 'Best','FontSize', 14);

subplot(2, 1, 2);
plot(time, vel_APM, 'b', 'LineWidth', 1.5); 
hold on;
plot(time, vel_ref, 'r--', 'LineWidth', 1.5); 
hold off;
xlabel('Time (sec)','FontSize', 16);
ylabel('Velocity (m/s)','FontSize', 16);
title('Velocity APM','FontSize', 18);
legend('Velocity APM', 'Reference Velocity', 'Location', 'Best','FontSize', 14);
ylim([min(vel_APM), max(vel_APM) + 1]);


% Plot the distance from APM to runner
figure
plot(time, d_p , 'b', 'LineWidth', 2);
%Plot the bound
yline(d_max, 'r', 'Distance max', 'LineWidth', 2,'FontSize', 14);
yline(d_min, 'r', 'Distance min', 'LineWidth', 2,'FontSize', 14);
xlabel('Time (sec)','FontSize', 18);
ylabel('Distance(m)','FontSize', 16);
title('Simulated Relative Distance','FontSize', 18);
legend('Distance between runner and APM','FontSize', 14)
ylim([3.5, 10]);


% Plot the speed runner
figure
plot(time, vel_d_shift, 'Color', [0.7, 0.7, 0.7], 'LineWidth', 2);
fill([time; flip(time)], [vel_d_shift; zeros(size(vel_d_shift))], [0.9, 0.9, 0.9], 'FaceAlpha', 0.3);

plot(time, vel_d_shift, 'Color', [0.7, 0.7, 0.7], 'LineWidth', 2);
fill([time; flip(time)], [vel_d_shift; zeros(size(vel_d_shift))], [0.9, 0.9, 0.9], 'FaceAlpha', 0.3);
hold on
plot(time, vel_run, 'b', 'LineWidth', 2);
hold on;
plot(time, vel_APM, 'r--', 'LineWidth', 1.5); 
hold off;
ylabel('Velocity runner(m)','FontSize', 16);
title('Simulated runner VS Squedule','FontSize', 18);
legend('Velocity sqedule','Velocity runner', 'Velocity APM', 'Location', 'Best','FontSize', 14);
xlabel('Time (sec)');

% % Plot the distance from APM to runner
figure
plot(time, p_e, 'b', 'LineWidth', 2);
yline(10, 'r', 'Max distance error', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'FontSize', 14);
yline(-10, 'r', 'Min distance error', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left','FontSize', 14);
xlabel('Time (sec)','FontSize', 16);
ylabel('Distance (m)','FontSize', 16);
title('Simulated Distance Error','FontSize', 18);
legend('Distance error (defined by the specified goals)', 'Location', 'Best','FontSize', 14);
% 
% % Plot the Blinking to the lamp
% figure
% plot(time, lamp, 'b', 'LineWidth', 2);
% xlabel('Time (sec)');
% ylabel('Signal');
% title('Simulated Lamp');
% legend('Signal to lamp');
% ylim([0, 1.1]);
% 





