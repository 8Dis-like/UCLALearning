function [f, ind] = generateMVPLinearParabolicBlendTrajectory(P, T, Acc)
%======================================================
% Author: Guanyu Chen
% Date: 12/31/2025
% Contact: gychen98@ucla.edu
% Input:
%   P - N Waypoints where N is the number of waypoints and n is the dimension of the waypoints
%   T - N Time instances
%   Acc - N-1 Desired acceleration magnitude
% Output:
%   f(t) - a function of t. Given time t, it will return the corresponding configuration nx1. 
%======================================================
    T_ini = T(1);    
    T = T - T_ini;
    ind = 1;    
    Acc = abs(Acc);
    F = {cell(length(T), 2),cell(length(T), 2),cell(length(T), 2)}; % left: linear, right: blend for position, velocity, acceleration
    % initialize F
    g = @(t) 0;
    for k = 1:3
        [F{k}{:}] = deal(g);
    end
    T_linear = zeros(1,length(T) - 1); % store the time duration for linear section
    T_blend = zeros(1,length(T)); % store the time duration for blend section
    Td = diff(T);
    if length(P) ~= length(T) || length(Acc) ~= length(T)
        disp("Generation failed! Check input dimensions!")
        ind = 0;
    else
        % each section is a blend+linear except for the last one.
        for i = 1:length(T) % for the ith blend
            if i == 1
                acc = Acc(i) .* sign(P(i+1)-P(i));
                T_blend(i) = Td(i) - sqrt(Td(i)^2-2*(P(i+1)-P(i))./acc);
                theta12dot = (P(i+1)-P(i))/(Td(i)-1/2*T_blend(i));
                % position
                fl = @(t) P(i) + theta12dot * (t - 1/2 * T_blend(i));
                fb = @(t) P(i) + 1/2 * theta12dot/T_blend(i) * t^2;
                % velocity
                fvl = @(t) theta12dot;
                fvb = @(t) theta12dot/T_blend(i) * t;
                % acceleration
                fal = @(t) 0 * theta12dot;
                fab = @(t) theta12dot/T_blend(i);
                F{1}{i,1} = fl;  F{1}{i,2} = fb;
                F{2}{i,1} = fvl; F{2}{i,2} = fvb;
                F{3}{i,1} = fal; F{3}{i,2} = fab;
            elseif i == length(T)
                % last segment
                acc = -Acc(:,i) .* sign(P(i)-P(i-1));
                T_blend(i) = Td(i-1) - sqrt(Td(i-1)^2+2*(P(i)-P(i-1))./acc);
                if i == 3
                    theta1dot = (P(i-1)-P(i-2))/(Td(i-2)-1/2*T_blend(i-2));
                else
                    theta1dot = (P(i-1)-P(i-2))/Td(i-2);
                end
                theta2dot = (P(i)-P(i-1))/(Td(i-1)-1/2*T_blend(i));
                acc12 = Acc(:,i) .* sign(theta2dot - theta1dot);
                T_blend(i-1) = (theta2dot - theta1dot)./acc12;
                if i==3
                    T_linear(i-2) = Td(i-2) - 1/2*T_blend(i-1) - T_blend(i-2);
                else
                    T_linear(i-2) = Td(i-2) - 1/2*T_blend(i-1) - 1/2*T_blend(i-2);
                end
                T_linear(i-1) = Td(i-1) - 1/2*T_blend(i-1) - T_blend(i);
                thetanm1ndot = (P(i)-P(i-1))/(Td(i-1)-1/2*T_blend(i));
                % T_linear(i) = Td(i) - T_blend(i) - 1/2*T_blend(i-1);
                % position
                fl = @(t) P(i-1) + thetanm1ndot * (1/2*T_blend(i-1)+T_linear(i-1)) + thetanm1ndot * (T_blend(i)) - 1/2 * thetanm1ndot/T_blend(i) * (T_blend(i))^2;
                fb = @(t) P(i-1) + thetanm1ndot * (1/2*T_blend(i-1)+T_linear(i-1)) + thetanm1ndot * (t) - 1/2 * thetanm1ndot/T_blend(i) * (t)^2;
                % velocity
                fvl = @(t) thetanm1ndot * 0;
                fvb = @(t) thetanm1ndot - thetanm1ndot/T_blend(i) * (t);
                % acceleration
                fal = @(t) 0 * theta12dot;
                fab = @(t) acc;
                F{1}{i,1} = fl;
                F{2}{i,1} = fvl;
                F{3}{i,1} = fal;
                F{1}{i,2} = fb;
                F{2}{i,2} = fvb;
                F{3}{i,2} = fab;

                % for i-1 th segment
                % position
                fl = @(t) P(i-1) + theta2dot * (t-1/2*T_blend(i-1));
                fb = @(t) P(i-2) + theta1dot * (1/2*T_blend(i-2)+T_linear(i-2)) + theta1dot * (t) + 1/2 * acc12 * (t)^2;
                % velocity
                fvl = @(t) theta2dot;
                fvb = @(t) theta1dot + acc12 * (t);
                % acceleration
                fal = @(t) 0 * theta2dot;
                fab = @(t) acc12;
                F{1}{i-1,1} = fl; F{1}{i-1,2} = fb;
                F{2}{i-1,1} = fvl; F{2}{i-1,2} = fvb;
                F{3}{i-1,1} = fal; F{3}{i-1,2} = fab;

            elseif i<length(T)-1 && i > 1
                
                theta2dot = (P(i+1)-P(i))/Td(i);
                if i ==2
                    theta1dot = (P(i)-P(i-1))/(Td(i-1)-1/2*T_blend(i-1));
                else
                    theta1dot = (P(i)-P(i-1))/Td(i-1);
                end
                acc = Acc(:,i) .* sign(theta2dot - theta1dot);
                T_blend(i) = (theta2dot - theta1dot)./acc;
                if i == 2
                    T_linear(i-1) = Td(i-1) - 1/2*T_blend(i) - T_blend(i-1);
                else
                    T_linear(i-1) = Td(i-1) - 1/2*T_blend(i) - 1/2*T_blend(i-1);
                end
                % position
                fl = @(t) P(i) + theta2dot * (t-1/2*T_blend(i));
                fb = @(t) P(i-1) + theta1dot * (1/2*T_blend(i-1)+T_linear(i-1)) + theta1dot * (t) + 1/2 * acc * (t)^2;
                
                % velocity
                fvl = @(t) theta2dot;
                fvb = @(t) theta1dot + acc * (t);
                % acceleration
                fal = @(t) 0 * theta2dot;
                fab = @(t) acc;
                F{1}{i,1} = fl; F{1}{i,2} = fb;
                F{2}{i,1} = fvl; F{2}{i,2} = fvb;
                F{3}{i,1} = fal; F{3}{i,2} = fab;
            end
        end
    end
    T_modified = T;
    T_modified(2:end-1) = T(2:end-1) - 1/2 * T_blend(2:end-1);
    T_modified(end) = T(end) - T_blend(end);
    T_modified = [T_modified, T(end)];
    if ~validateBlendtime(T_blend, Td)
        ind = 0;
    end
    f = @getConfiguration;
    function y = getConfiguration(t)
        t = t - T_ini;
        if t == T(1)
            idx = 1;
        else
            idx = find(T_modified < t, 1, 'last');
        end

        t = t - T_modified(idx);
        if t <= T_blend(idx)
            f_pos = F{1}{idx,2};
            f_vel = F{2}{idx,2};
            f_acc = F{3}{idx,2};
        else
            f_pos = F{1}{idx,1};
            f_vel = F{2}{idx,1};
            f_acc = F{3}{idx,1};
        end
        
        y = [f_pos(t);f_vel(t);f_acc(t)];

    end

end

function ind2 = validateBlendtime(T_blend, Td)
    ind2 = 1;
    for i = 1: length(T_blend) - 1
        if 1/2*T_blend(i)+1/2*T_blend(i+1)>Td(i)
            disp("The duration for " + i + " and " + (i+1) + " blend is too long. Try greater acceleration for these blends!")
            ind2 = 0;
        end
    end

end
