%% Gimbal Control System Full Analysis Script
clc; clear; close all;

%% 1. Physical Parameters and Transfer Function
J = 0.0012; % Moment of inertia of gimbal + camera (kg*m^2)
b = 0.015;  % Rotational joint friction coefficient (N*m*s)
Kt = 0.05;  % Motor torque constant (N*m/A)

s = tf('s');
G_plant = Kt / (J*s^2 + b*s); % Transfer function of motor and mechanical system

%% 2. PID Parameters Defined in C Code
Kp = 2.5;
Ki = 0.05;
Kd = 0.8;
C_pid = pid(Kp, Ki, Kd);

% Closed-loop System
sys_cl = feedback(C_pid * G_plant, 1);

%% 3. Time-Domain Simulation
t = 0:0.001:2; % 2-second simulation time
step_input = 10; % 10-degree sudden tilt/disturbance

[y_clean, t_out] = step(step_input * sys_cl, t);

% Adding Sensor Noise (MPU-6050 Accelerometer Vibration Simulation)
noise = 0.5 * randn(size(t_out)); 
y_noisy = y_clean + noise';

%% 4. Graphical Visualization
figure('Name', 'Gimbal PID and Sensor Analysis', 'NumberTitle', 'off');

% Plot 1: Ideal vs. Noisy Angular Response
subplot(2,1,1); plot(t_out, y_clean, 'b-', 'LineWidth', 2); hold on;
plot(t_out, y_noisy, 'r:', 'LineWidth', 0.8);
title('Gimbal Tilt Angle Response (10° Reference)');
xlabel('Time (s)'); ylabel('Angle (Degrees)');
legend('Ideal PID Response', 'Noisy Sensor Reading');
grid on;

% Plot 2: Error Analysis
subplot(2,1,2);
error_val = step_input - y_clean;
plot(t_out, error_val, 'r-', 'LineWidth', 1.5);
title('System Control Error');
xlabel('Time (s)'); ylabel('Error (Degrees)');
grid on;