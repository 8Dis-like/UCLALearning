%% Simple Validation Script for Linear Parabolic Trajectory
clear; clc; close all;

%% 1. Define Inputs
% Waypoints (2 Dimensions, 4 Points)
% Row 1: Dimension 1 (e.g., X)
% Row 2: Dimension 2 (e.g., Y)
P = [0, 10, 30, 15, 10]; 

% Time instances
T = [0, 2, 4, 6, 8]; 

% Acceleration Limit
Acc = [10 10 10 20 20]; 

%% 2. Generate Trajectory
% Make sure generateLinearParabolicTrajectory.m is in your folder
[f, ind] = generateMVPLinearParabolicBlendTrajectory(P, T, Acc);
ind
%% 3. Sampling for Plotting
dt = 0.01; 
t_seq = T(1):dt:T(end);
results = zeros(3, length(t_seq)); % [pos; vel; acc] (2 dims each -> 6 rows)

for i = 1:length(t_seq)
    results(:, i) = f(t_seq(i));
end

% Extract data
pos_data = results(1, :);
vel_data = results(2, :);
acc_data = results(3, :);

%% 4. Plot Results
figure('Name', 'Trajectory Profiles', 'Color', 'w', 'Position', [200, 100, 800, 800]);

% --- Subplot 1: Position ---
subplot(3,1,1);
plot(t_seq, pos_data, 'LineWidth', 2); hold on;
% Plot original waypoints P at times T to see the tracking
set(gca, 'ColorOrderIndex', 1); % Reset color cycle to match lines
plot(T, P, 'o', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'w'); 
grid on;
title('Position vs Time (with Waypoints)');
ylabel('Position [m]');
legend('Interpolated','Waypoints');

% --- Subplot 2: Velocity ---
subplot(3,1,2);
plot(t_seq, vel_data, 'LineWidth', 2);
grid on;
title('Velocity vs Time');
ylabel('Velocity [m/s]');

% --- Subplot 3: Acceleration ---
subplot(3,1,3);
plot(t_seq, acc_data, 'LineWidth', 2);
grid on;
title('Acceleration vs Time');
ylabel('Accel [m/s^2]');
xlabel('Time [s]');