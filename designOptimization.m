function [L1_star, L2_star, maxVal, L_pair, C] = designOptimization(max_lengths, workspace, settings)
    % Inputs: 
        % max_lengths [L1_max, L2_max] 
        % workspace [w, h, d] 
        % settings .delL1 .delL2 .delx .dely 
    % Outputs: 
        % optimal lengths [L1_star, L2_star] 
        % maxVal maximum C value

    L_pair = [];
    C = [];

    % ===== Discretized parameter vectors =====
    L1_vec = 0.01:settings.delL1:max_lengths(1);
    L2_vec = 0.01:settings.delL2:max_lengths(2);
    x_vec  = workspace(3):settings.delx:workspace(1)+workspace(3);
    y_vec  = -workspace(2)/2:settings.dely:workspace(2)/2;

    % ===== Total progress steps (update per x iteration only) =====
    totalSteps = length(L1_vec) * length(L2_vec) * length(x_vec);
    stepCount = 0;

    for L1 = L1_vec
        for L2 = L2_vec
    
            if checkreachable(L1, L2, workspace)
    
                K = [];  % store kappa
    
                for x = x_vec
                    % ====================================================
                    % Update progress counter
                    stepCount = stepCount + 1;
                    % Update every 20 steps to avoid slowing down
                    if mod(stepCount,100) == 0
                        fprintf(repmat('\b',1,100)); 
                        fprintf('Processing...\nProgress: %6.2f %%', ...
                            100*stepCount/totalSteps);
                    end
                    % ====================================================
    
                    for y = y_vec
                        % use IK to get joint configuration
                        q_sol = inverseKinematics(x,y,[L1,L2]);
                        q = q_sol{1};
                        
                        % calculate Jacobian matrix at the certain joint
                        % configuration
                        J = getJacobian(q(1),q(2));

                        % calculate eigenvalues of JJ'
                        eigenvalues = eig(J*J');
                        eigenvalues_sorted = sort(eigenvalues,'descend');
                        
                        % find out the max and min eigenvalues
                        lambda_max = eigenvalues_sorted(1);
                        lambda_min = eigenvalues_sorted(2);  
                        
                        % calculate condition number
                        condition_number = lambda_min/lambda_max;
                        
                        K(end+1) = condition_number;
    
                    end
                end
                % calculate kappa sum and min kappa
                sum_kappa = sum(K);
                kappa_min = min(K);
                
                % assemble C and L_pair
                C(end+1) = sum_kappa*kappa_min/(L1^3+L2^3);
                L_pair(end+1,:) = [L1,L2];
    
            else
                stepCount = stepCount + length(x_vec);
            end
    
        end
    end

    fprintf('\nOptimization completed.\n');

    % Extract optimal result
    [maxVal, idx] = max(C);
    L1_star = L_pair(idx,1);
    L2_star = L_pair(idx,2);

end

function J = getJacobian(q1,q2)
   J =  [- 2*sin(q1 + q2) - sin(q1), -2*sin(q1 + q2);
       2*cos(q1 + q2) + cos(q1),  2*cos(q1 + q2);
                         0,               0;
                         0,               0;
                         0,               0;
                         1,               1];
end

function ind = checkreachable(L1, L2, workspace)
    if L1+L2 >= sqrt((workspace(1)+workspace(3))^2 + 0.25*workspace(2)^2) && abs(L1-L2) <= workspace(3)
        ind = 1;
    else
        ind = 0;
    end
end

function [q] = inverseKinematics(px, py, l)
            q = {};
            c2 = (px^2 + py^2 - l(1)^2-l(2)^2)/(2*l(1)*l(2));
            c2 = clip(c2, -1 ,1);
            s2_1 = sqrt(1-c2^2);
            s2_2 = -sqrt(1-c2^2);
            k1 = l(1) + l(2) * c2;
            k2_1 = l(2) * s2_1;
            k2_2 = l(2) * s2_2;
            
            q{2} = [atan2(py,px)-atan2(k2_1,k1);atan2(py,px)];
            q{1} = [atan2(py,px)-atan2(k2_2,k1);atan2(py,px)];
end