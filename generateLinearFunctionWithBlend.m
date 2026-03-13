% generateLinearFunctionWithBlend
function [Pos, Vel, Acc, ind] = generateLinearFunctionWithBlend(x_ini, x_f, t_ini, t_f, a, type, N)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input: 
% x_ini - Scalar initial position
% x_f - Scalar final position
% t_ini - Scalar initial time
% t_f - Scalar final time
% a - acceleration or blend time
% type - "acc" or "time"
% N - Scalar number of points to be generated

% Output:
% Pos - Nx1 positions
% Vel - Nx1 velocities
% Acc - Nx1 accelerations
% ind - Scalar indicating whether the generation is successful or not, 1 or 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the output
Pos = zeros(N,1);
Vel = zeros(N,1);
Acc = zeros(N,1);
T = linspace(t_ini, t_f, N);
% Check the type acc or time
% t_ini----first blend----tb1----linear----tb2-----last blend----t_f
if type == "acc"
    % find out the blend time
    % tb1 = ?  % for the first blend
    % tb2 = ?  % for the last blend
    acc = a;
elseif type == "time"
    % find out the blend time
    tb1 = t_ini + a;  % for the first blend
    tb2 = t_f - a;  % for the last blend 
    % acc = ?;
else
    disp("Input a valid type!")
    ind = 0;  % generation failed.
    return
end

ind = 1;
for i = 1:N
    t = T(i);
    
    % Check if t is in the first blend, linear section, the last blend using tb1 and tb2.
    % if (in the first blend)
    % the displacement equation for the first blend

    % the velocity equation for the first blend

    % the acceleration equation for the first blend

    % elseif (in the linear section)
    % the displacement equation for the first blend

    % the velocity equation for the first blend

    % the acceleration equation for the first blend

    % else
    % the displacement equation for the first blend

    % the velocity equation for the first blend

    % the acceleration equation for the first blend

    % end

    % store the position, velocity, and acceleration at time instance t
    % Pos(i) = ?
    % Vel(i) = ?
    % Acc(i) = ?

end

% check if the blends have overlaps.
if 2*(tb1 - t_ini) > t_f - t_ini  % (tb1 - t_ini) is the blend duration (which is "a" when "type" = "time")
    ind = 0;
end

end