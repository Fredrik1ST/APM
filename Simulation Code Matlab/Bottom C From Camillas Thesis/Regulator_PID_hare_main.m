%% Simulation of Speed regulation for the autonom rabbit 
% GPS modue: RTK U-blox ZED-F9P
clear; clc; close all;

% Most of this code is copied from Camilla Kvamme's thesis 
% "Utvikling av en autonom hare for løp på bane"

%% Initialization
% Excel file name of the refrence speed

load('test1_vd.mat');
data_speed = simdata(:,3);          % Loaded from earlier simdata
time_data = simdata(:,1); 

% Simulation parameters
Ts = 1/10;                          % 20 Hz Sampling Time
t_prev = 0;                         % Previous time 
t = 0;                              % Time constant
j = 1;                              % Counter that updates the time from data

Kp = 80;                             % Propotional gain
Ki = 1251;                           % Integral gain
Kd = 0;                              % Derivate gain

%Parameter used during simulation tests
% Kp = 10;                           % Propotional gain
% Ki = 145;                          % Integral gain
% Kd = 0.0;                          % Derivate gain

% Anti-windup limits
max_int = 80;                       % Maximum value for the integral term
min_int = -80;                      % Minimum value for the integral term

% Initialization velocity
v_ref = 0;                          % Desired velocity from camara regulation
v_APM = 0;                          % Current speed APM

% Initialization position
p_ref = 0;                          % Desired position
p_APM = 0;                          % Current position APM 

% Initialization GPS-terms
e_gps_const = 0.05*0.277777778;    % GPS Error
e_p = 0;                            % Propotional term
e_int = 0;                          % Integral term
e_d = 0;                            % Derivate term
e_prev = 0;                         % Previous error for the derivative term

% Initialization filter
y_k = 0;                            % Last filtred value
alpha = 1;

%% For-loop for the system
N = time_data(end,:)/Ts;            % Timestep for loop
simdata = zeros(length(N), 5);      % Table of simulation data
for i = 1:N
    
    % Update v_ref when the system get an input from the refrence speed 
    if t >= (time_data(j))
        v_ref = data_speed(j);
        j = j + 1;      
    end
    % If refrence speed  is negative or zero, set current speed to zero 
    if v_ref <= 0
        v_APM = 0;
    end

    % Error GPS
    e_gps = (rand() - e_gps_const) * 2 * e_gps_const;

    % Lowpass filter 
    v_f = (1 - alpha) * y_k + alpha * (v_APM + e_gps); %Inserting GPS error

    if(v_f < 0.05) % If speed is lover than the noise from GPS
        v_f = 0;
    end
    y_k = v_f; 

    % Update PID error parameters
    e_p = v_ref - v_f ; 
    e_int = e_int + e_prev * Ts; 
    e_d = (e_p - e_prev)/Ts;     
    e_prev = e_p; 
    
    % Anti-windup 
    e_int = max(min(e_int, max_int), min_int);

    % PID regulator
    u = Kp * e_p + Kd * e_d + Ki * e_int; 
    
    % Input voltage to the dc motor
    v_APM = plant_DCmotor(Ts , [u u], v_ref);

    % Update position
    p_APM =  p_APM + v_APM * Ts;
    p_ref = p_ref + v_ref *Ts;    

    % Update time
    t = t + Ts;
    
    % Store simulation data in a table 
    simdata(i,:) = [t/60 v_APM v_ref p_APM p_ref]; 

    if j > length(data_speed)
        break;
    end
    
end

%% Plot
time      = simdata(:,1);                % min
v_c       = simdata(:,2);                % m/s
v_d       = simdata(:,3);                % m/s
p_c       = simdata(:,4);                % m
p_p       = simdata(:,5);                % m

figure;
% Plot for velocity
subplot(2, 1, 1);
plot(time, v_c, 'b', 'LineWidth', 2);
hold on;
plot(time, v_d, 'r--', 'LineWidth', 2);
xlabel('Time (Min)','FontSize', 16);
ylabel('Velocity (m/s)','FontSize', 16);
title('Simulated velocity','FontSize', 18);
legend('Actual Velocity', 'Desired Velocity','FontSize', 14);

% Plot for position
subplot(2, 1, 2);
plot(time, p_c, 'b', 'LineWidth', 2);
hold on;
plot(time, p_p, 'r--', 'LineWidth', 2);
xlabel('Time (Min)','FontSize', 16);
ylabel('Position (m)','FontSize', 16);
title('Simulated position','FontSize', 18);
legend('APM position', 'Desired Position','FontSize', 14);