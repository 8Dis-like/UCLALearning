clear
clc
close all

num_joints = 2;
syms q [num_joints 1] real
syms qd [num_joints 1] real
syms qdd [num_joints 1] real
syms Ixx [num_joints 1] real
syms Iyy [num_joints 1] real
syms Izz [num_joints 1] real
syms lcom [num_joints 1] real
syms m [num_joints 1] real
syms L1 L2 g real
syms fe [3 1] real
syms ne [3 1] real

T{1} = [cos(q(1)) -sin(q(1)) 0 0;
        sin(q(1)) cos(q(1)) 0 0;
        0 0 1 0;0 0 0 1];

T{2} = [cos(q(2)) -sin(q(2)) 0 L1;
        sin(q(2)) cos(q(2)) 0 0;
        0 0 1 0;0 0 0 1];

T{3} = [1 0 0 L2;
        0 1 0 0;
        0 0 1 0;0 0 0 1];

num_frames = length(T);
num_joints = num_frames - 1; 

Ic = {};
Pc = {};
for i=1:num_joints
    Ic{i} = diag([Ixx(i),Iyy(i),Izz(i)]);
    Pc{i} = [lcom(i);0;0];
end

% Begin iteration
w = {}; % 0w0 1w1 2w2 ...
w{1} = zeros(3, 1);  
w_dot = {}; % 0w_dot0 1w_dot1 ...
w_dot{1} = zeros(3, 1);  
v_dot = {};   
v_dot{1} = [0;0;g]; % 0w_dot0 1w_dot1 ...
f = {};  % 1f1 2f2 ....
n = {};  % 1n1 2n2 ....
f{num_frames} = fe;
n{num_frames} = ne;
F = {};  % 1F1 2F2 ....
N = {};  % 1N1 2N2 ....

for i=1:num_frames-1
    Riim1 = T{i}(1:3,1:3)'; % Rotation matrix 
    Pim1i = T{i}(1:3,4);  % translation
    w{i+1} = ;  % i+1 w i+1
    w_dot{i+1} = ;  % i+1 w_dot i+1
    v_dot{i+1} = ; % i+1 v_dot i+1
    vc_dot{i+1} = ;  % i+1 v_dot ci+1
    
    F{i} = ;  
    N{i} = ;
end

% inward iteration
tau = sym(zeros(num_frames-1, 1));
for i=num_frames-1:-1:1
    f{i} = ;
    n{i} = ;
    tau(i) = n{i}(3);
end

G = simplify(subs(tau, [qd; qdd], [zeros(num_joints, 1); zeros(num_joints, 1)]))
M = simplify(jacobian(tau, qdd))
C = simplify(subs(tau, [g; qdd], [0; zeros(num_joints, 1)]))

J = simplify(jacobian(tau, [fe;ne]))


