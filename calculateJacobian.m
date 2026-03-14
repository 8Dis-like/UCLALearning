function J = calculateJacobian(T, joint, joint_type, method)
% Inputs:
    % T - a cell containing n transformation matrices of joints w.r.t previous frame in sequence 
    % joint - joint of interest
    % joint_type - n-dim list [0 for prismatic; 1 for revolute] 
    % method - "explicit" "vp" "fp"
% Outputs:
    % J - Jacobian matrix (6xn)
    T0j = eye(4);
    for i = 1:joint
        T0j = T0j * T{i}; % Trans. matrix from base to the joint of interest
    end

    if method == "vp"
        % vp method
        syms theta_dot [1 joint] real;
        % For angular velocity
        wii{1} = [0;0;0];
        for i = 0:joint-1
            wii{i+2} = T{i+1}(1:3,1:3)' * wii{i+1} + [0;0;joint_type(i+1)*theta_dot(i+1)];
        end
        % For linear velocity
        vii{1} = [0;0;0];
        for i = 0:joint-1
            vii{i+2} = T{i+1}(1:3,1:3)' * (cross(wii{i+1},T{i+1}(1:3,4)) + vii{i+1}) + [0;0;(~joint_type(i+1))*theta_dot(i+1)];
        end
        wii = wii{end};
        vii = vii{end};
        % Extract the Jacobian matrix w.r.t to frame of the joint of
        % interest
        [J_v, ~] = equationsToMatrix(vii, theta_dot);
        [J_w, ~] = equationsToMatrix(wii, theta_dot);
        % Convert the Jacobian matrix back to the base frame
        J = [T0j(1:3,1:3) zeros(3);zeros(3) T0j(1:3,1:3)] * [J_v;J_w];
    elseif method == "fp"
        % fp method
        syms wip1ip1 [6 1] real
        fip1ip1 = wip1ip1(1:3);
        nip1ip1 = wip1ip1(4:6);
        
        nii = cell(joint, 1);
        nii{joint} = nip1ip1;
        fii = cell(joint, 1);
        fii{joint} = fip1ip1;
        for i = joint-1:-1:1
            fii{i} = T{i+1}(1:3,1:3) * fii{i+1};
            nii{i} = T{i+1}(1:3,1:3) * nii{i+1} + cross(T{i+1}(1:3,4),fii{i});
        end
        
        taui = sym(zeros(joint,1));
        for i = 1: joint
            if joint_type(i) == 0
                % for prismatic joint
                taui(i) = dot(fii{i}, [0;0;1]);
            else
                % for revolute joint
                taui(i) = dot(nii{i}, [0;0;1]);
            end
        end
        [NJT, ~] = equationsToMatrix(taui, wip1ip1);
        NJ = NJT';
        J = [T0j(1:3,1:3) zeros(3);zeros(3) T0j(1:3,1:3)] * NJ;
    elseif method == "explicit"
        % explicit method
        T0 = eye(4);  % w.r.t base frame
        for i = 1:joint
            T0 = T0*T{i};
            if joint_type(i) == 0
                % For prismatic
                J(:, i) = [T0(1:3,3); 0*T0(1:3,3)]; 
            else
                % For revolute
                J(:, i) = [cross(T0(1:3,3), T0j(1:3,4)-T0(1:3,4)); T0(1:3,3)];
            end
        end
    end
end
