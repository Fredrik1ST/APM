function [v_c] = plant_DCmotor(Ts, u, v_ref)
% [v_c] = plant_DCmotor(Ts, u, v_ref)
% Calculates the speed (km/h) for the rabbit
%
% Ts = Sampling time
% u = Contoll law from the PID regulator
% v_ref = Refrence speed

persistent x_prev

% Initial previuous states
if isempty(x_prev)
    x_prev = 0; 
end

% Parameters used in Simulation of the system
% % Defining motor parameters
% J = 0.02;          % Kgm^2
% b = 0.2;           % N.m/(rad/sec)
% Kt = 1.2;          % N.m/Amp
% Kb = 1;            % V-sec/rad
% R = 0.0075;        % ohm
% L = 0.3;           % mH 
% r = 0.06;          % m
% 
% 
% t=0:0.05:Ts;       % Time sample
% % Transfer function for the DC motor
% s = tf('s');
% motor_sys = Kt/((J*s+b)*(L*s+R)+(Kt*Kb)^2);
% [y,x] = lsim(motor_sys, u, t, x_prev);


%Transferfunction found after driving real--world APM 
t=0:Ts:Ts;
s = tf('s');
motor_sys = (0.05236*s + 0.0001726)/(s^2 + 0.0978*s+0.0003001);
 [y,x] = lsim(motor_sys, u, t, x_prev);

% Update previous state
x_prev = x(2);

% Linear velocity
 v_c = y(2); % *r; only for simulation tests not real world

end