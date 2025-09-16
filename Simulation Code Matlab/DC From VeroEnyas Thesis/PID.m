clear; clc; close all;

% Most of this code is copied from Camilla Kvamme's thesis 
% "Utvikling av en autonom hare for løp på bane", and Veronica's and 
% Enya's thesis "Development of an Autonomous Rabbit for Running on a Track"

%% Initialization
% Excel file name of the reference speed
load('test1_vd.mat');
data_speed = simdata(:,3); %Loaded from earlier simdata
time_data = simdata(:,1);

% Simulation parameters
Ts = 1/10; % 20 Hz Sampling Time
t = 0; % Time constant
j = 1; % Counter that updates the time from dat
epsilon = 0.1; % Small velocity threshold for stopping
max_acceleration = 0.1; % Max acceleration m/Ts

% Initialization velocity
v_ref = 0;
v_runner = 0;
v_APM = 0;

% Initialization position
p_ref = 0;              % Desired position
p_APM = 0;              % Current position APM 

% Initialization of Distance
d_max = 5;
d_min = 4;
d_ref = (d_max + d_min) / 2; % Desired distance
d_runner = (d_max + d_min) / 2;

% PID Parameters Distance Control
Kp_ref = 1.5; % Proportional gain
Ki_ref = 0.0; % Integral gain
Kd_ref = 0.06; % Derivative gain

%In Simulations
% Kp_ref = 2; % Proportional gain
% Ki_ref = 0; % Integral gain
% Kd_ref = 0.9; % Derivative gain

integral_error = 0; % Integral error
previous_error = 0; % Previous error for derivative calculation
d_error = 0;

% Camilla PID Controller Speed Control
Kp = 80;                % Propotional gain
Ki = 1251;              % Integral gain
Kd = 0.0;               % Derivate gain

% Anti-windup limits
max_int = 70;           % Maximum value for the integral term
min_int = -70;          % Minimum value for the integral term

% Initialization GPS-terms
e_gps_const = 0.05;     % GPS Error
e_p = 0;                % Propotional term
e_int = 0;              % Integral term
e_d = 0;                % Derivate term
e_prev = 0;             % Previous error for the derivative term

% Initialization filter
y_k = 0;                % Last filtred value
alpha = 1;
v_f = 0;                % Filtred value

% Gamma-beta pid
gamma = 1;
beta = 1;
%% For-loop
N = time_data(end,:)/Ts; % Timestep for loop
simdata = zeros(length(N), 8); % Table of simulation data
for i = 1:N

    if j > length(data_speed)
        break;
    end

    % Update v_ref to the lap time schedule
    if t >= (time_data(j))
        v0_ref = data_speed(j); % v_0,ref is desired speed
        v_runner = data_speed(j) ;%- (rand() - max_acceleration); % Measurements of runner's velocity 
        if(v_runner < 0 || v0_ref == 0)
            v_runner = 0;
        end

        j = j + 1;
    end

    % Calculate distance error
       d_error = d_ref - d_runner;

    if v_runner < epsilon
        % Stop the APM if the runner's velocity is very small
        v_ref = 0;
        previous_error = d_error;
    else

        % PID Control for calculating speed to make up for deviation in
        % distance.
        integral_error = integral_error + d_error * Ts;
        derivative_error = (d_error - previous_error) / Ts;
        v_ref = v_APM + Kp_ref * d_error + Ki_ref * integral_error + Kd_ref * derivative_error;

        previous_error = d_error;
    end

    % Update Distance (Euler)
    d_dot = v_ref - v_runner;
    d_runner = d_runner + d_dot * Ts;

    % Keep the distance from being negative
    if d_runner <= 0
        d_runner = 0;
    end 
    
    % Lowpass filter 
    v_f = (1 - alpha) * y_k + alpha * (v_ref); 
    if(v_f < 0.05) % If speed is lover than the noise from GPS
        v_f = 0;
    end
    y_k = v_f; 

    % Update PID error parameters
    e_p = v_f - v_APM; 
    e_int = e_int + e_prev * Ts; 
    e_d = (e_p - e_prev)/Ts;     
    e_prev = e_p; 
    
    % Anti-windup (limit the integral term)
    e_int = max(min(e_int, max_int), min_int);

    % PID regulator
    u = Kp * e_p + Kd * e_d + Ki * e_int; 
    
    % Input voltage to the dc motor
    v_APM = plant_DCmotor(Ts , [u u], v_ref);
    
    % Prevent speed to be negative
    if v_APM <= 0
        v_APM = 0;
    end

    % Update position
    p_APM = p_APM + v_APM * Ts;
    p_ref = p_ref + v_ref *Ts; 
    
    % Update time
    t = t + Ts;

    % Store simulation data in a table
    simdata(i,:) = [t v0_ref v_runner v_APM d_runner d_error p_APM p_ref];
end


%% Plot
time      = simdata(:,1);                % sec
v_0       = simdata(:,2);                % m/s
v_r       = simdata(:,3);                % m/s
v_APM     = simdata(:,4);                % m/s
d_r       = simdata(:,5);                % m
d_er      = simdata(:,6);                % m
p_c       = simdata(:,7);                % m
p_p       = simdata(:,8);                % m

figure;
% Plot for velocity
% subplot(2, 1, 1);
plot(time, v_APM, 'b', 'LineWidth', 1.5);
hold on;
plot(time, v_0, 'r--', 'LineWidth', 1.5);
xlabel('Time (sec)','FontSize', 16);
ylabel('Velocity (m/s)','FontSize', 16);
title('Simulated velocity','FontSize', 18);
legend('Actual Velocity', 'Desired Velocity','FontSize', 14);

% % Plot for position
% subplot(2, 1, 2);
% plot(time, p_c, 'b', 'LineWidth', 2);
% hold on;
% plot(time, p_p, 'r--', 'LineWidth', 2);
% xlabel('Time (Min)');
% ylabel('Position (m)');
% title('Simulated position');
% legend('APM position', 'Desired Position');

% Plot the distance from APM to runner
figure
plot(time, d_r, 'b', 'LineWidth', 2);
%Plot the bound
yline(d_max, 'r', 'Distance max','FontSize', 16);
yline(d_min, 'r', 'Distance min','FontSize', 16);
xlabel('Time (sec)','FontSize', 16);
ylabel('Distance(m)','FontSize', 16);
title('Simulated Relative Distance','FontSize', 18);
legend('Distance between runner and APM','FontSize', 14)
ylim([3.5, 5.5]);

