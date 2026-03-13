clear
clc
close all

q0 = 0;
qf = 1.2532;
t0 = 0;
tf = 3;
v0 = 0;
vf = 0;
a0 = 0;
af = 0;
N = 100;
t = linspace(t0,tf,N);
[pos, vel, acc, jerk] = quanticTrajectory(t0, tf, q0, qf, v0, vf, a0, af, N);

figure('Color', 'w', 'Position', [100, 100, 800, 600]);

% Subplot 1: Trajectory Position
subplot(2,2,1);
plot(t, pos, 'm*', 'MarkerSize', 4); %
title('Trajectory Position for Joint 1', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Position (rad)');
grid on;
xlim([0 3]);
ylim([0 1.5]);

% Subplot 2: Trajectory Velocity
subplot(2,2,2);
plot(t, vel, 'r*', 'MarkerSize', 4); % 
title('Trajectory Velocity for Joint 1', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Velocity (rad/s)');
grid on;
xlim([0 3]);
% ylim([0 0.8]); % 

% Subplot 3: Trajectory Acceleration
subplot(2,2,3);
plot(t, acc, 'g*', 'MarkerSize', 4); % 
title('Trajectory Acceleration for Joint 1', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Acceleration (rad/s^2)');
grid on;
xlim([0 3]);
% ylim([-1 1]);

% Subplot 4: Trajectory Jerk
subplot(2,2,4);
plot(t, jerk, 'k*', 'MarkerSize', 4); %
title('Trajectory Jerk for Joint 1', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Jerk (rad/s^3)');
grid on;
xlim([0 3]);


q0 = pi/2;
qf = 0.6351;
[pos, vel, acc, jerk] = quanticTrajectory(t0, tf, q0, qf, v0, vf, a0, af, N);
figure('Color', 'w', 'Position', [100, 100, 800, 600]);

% Subplot 1: Trajectory Position
subplot(2,2,1);
plot(t, pos, 'm*', 'MarkerSize', 4); %
title('Trajectory Position for Joint 2', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Position (rad)');
grid on;
xlim([0 3]);
ylim([0.5 1.6]);

% Subplot 2: Trajectory Velocity
subplot(2,2,2);
plot(t, vel, 'r*', 'MarkerSize', 4); % 
title('Trajectory Velocity for Joint 2', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Velocity (rad/s)');
grid on;
xlim([0 3]);
% ylim([0 0.8]); % 

% Subplot 3: Trajectory Acceleration
subplot(2,2,3);
plot(t, acc, 'g*', 'MarkerSize', 4); % 
title('Trajectory Acceleration for Joint 2', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Acceleration (rad/s^2)');
grid on;
xlim([0 3]);
% ylim([-1 1]);

% Subplot 4: Trajectory Jerk
subplot(2,2,4);
plot(t, jerk, 'k*', 'MarkerSize', 4); %
title('Trajectory Jerk for Joint 2', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Time(s)');
ylabel('Jerk (rad/s^3)');
grid on;
xlim([0 3]);