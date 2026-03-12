clear all; clc; close all;
%% Define Robot
% Lets define a 2R robot using robotic toolbox!
fprintf('* ========== Modified DH parameter ========== *')
syms l1 l2  % Create symbolic variables and functions
%: cannot specify 'theta' for revolute link
%: cannot specify 'd' for primatic link
L1 = Link('revolute','d',0,'a',0,'alpha',0,'modified')
%: Link([theta, d, a, alpha, offset],'modified/standard');
L1_2 = Link([0 0 0 0],'modified') % Another way of defining a link
L1_3 = Link([0 0 0 0 1 90],'modified')
L1_4 = Revolute('d',0,'a',0,'alpha',0,'modified')
L1_5 = RevoluteMDH('d',0,'a',0,'alpha',0)
L1_6 = Prismatic('theta',0,'a',0,'alpha',0,'modified')
L1_7 = PrismaticMDH('theta',0,'a',0,'alpha',0)

% joint type and offset
 
L2 = Link('revolute','d',0,'a',l1,'alpha',0,'modified')

L3 = Link('d',0,'a',l2,'alpha',0,'modified')  % default: revolute
L3_2 = Link('revolute','d',0,'a',l2,'alpha',0,'modified')
L3_p = Link('prismatic','a',l2,'alpha',0,'theta', 0,'modified') 

Robot_Mod_DH = SerialLink([L1 L2 L3], 'name', '2D-RR Modified DH parameter') % Combine Link objects together to form a Robot Object
%% Forward Kinematics: Robotname.fkine()
syms t1 t2;
t3 = 0;
th = [t1, t2, t3];
FK1 = Robot_Mod_DH.fkine(th)

%% Get Homogeneous Transformation Matrix: Robotname.A()
% Transformation from frame 0 to 1
fprintf('* T0_1')
T01 = Robot_Mod_DH.A([1], th)

% Transformation from frame 1 to 2
fprintf('* T1_2')
T12 = Robot_Mod_DH.A([2], th)

% Transformation from frame 0 to 2
fprintf('* T0_2')
T02 = Robot_Mod_DH.A([1, 2], th)
% Transformation from frame 0 to 3
fprintf('* T0_3')
T03 = Robot_Mod_DH.A([1, 2, 3], th)

%% Plot robots: Robotname.plot()
fprintf('* ========== Plot robots ========== *')
% Need numerical values
l1 = 0.1;
l2 = 0.1;
l2_2 = 0.3;

L(1) = Link('revolute','d',0,'a',0,'alpha',0,'modified') ;
L(2) = Link('revolute','d',0,'a',l1,'alpha',0,'modified');
L(3) = Link('revolute','d',0,'a',l2,'alpha',0,'modified');

Robot_1 = SerialLink(L, 'name', '2D-RR Robot - configuration 1');
Robot_2 = SerialLink(L, 'name', '2D-RR Robot - configuration 2');

L2(1) = L(1); 
L2(2) = L(2);
L2(3) = Link('revolute','d',0,'a',l2_2,'alpha',0,'modified');
Robot_3 = SerialLink(L2, 'name', '2D-RR Roboot - link value- change')

figure()
Robot_1.plot([0 0 0]) %[t1 t2 t3]
figure()
Robot_2.plot([pi/2 pi/2 0])
figure()
Robot_3.plot([0 0 0])

%% Simulation
l1 = 0.1; 
l2 = 0.1;

time = [0 1 2 3 4];  
t1 = [0 pi/10 pi/8 pi/4 pi/2];
t2 = [0 pi/10 pi/6 pi/4 pi/2];

figure()
subplot(2,1,1)
plot(time,t1); xlabel('time'); ylabel('theta1');
subplot(2,1,2)
plot(time,t2); xlabel('time'); ylabel('theta2');

figure()
for i = 1:5
    Robot_1.plot([t1(i) t2(i) 0],'delay',0.5)
end

%% t2r and .t
%t2r : to extract the rotation matrix from the transformation matrix 
%.t : to extract the translation matrix from the transformation matrix 
fprintf('* ========== Examples ========== *')
syms l1 l2 t1 t2 c1 c2


fprintf('* Tranformation matrix T')
Rot_m = t2r(T03)
tlans_m = T03.t
%% Useful functions in MATLAB: simplify , collect, equationToMatrix, Transpose, cross
T03_simp = simplify(T03)
equation = c1*t1 + c2*t2 + c2*t1 + c2*t2
%%
collect = collect(equation,[t1 t2]) % combine the like terms
%%
e2m = equationsToMatrix(equation,[t1 t2])

%% Transpose
A = [1 + 2 * j 2; 3 4*j]
A' % Complex Conjugate
A.' 
transpose(A) % A.' = transpose(A) 


%% Puma 560 simulation
clear all;
close all;
clc;
% Define Puma 560 given the modified DH table
syms t1 t2 t3 t4 t5 t6 a2 a3 d3 d4 % Define all variables
L1 = [];
L2 = [];
L3 = [];
L4 = [];
L5 = [];
L6 = [];

th = [t1 t2 t3 t4 t5 t6];
Puma560 = SerialLink([L1 L2 L3 L4 L5 L6], 'name','Puma 560')

% Forward Kinematics
Puma_fk = simplify() % Fill this out 

%% Puma Simulation
th = [0 0 0 0 0 0];
a2 = 0.4318; a3 = 0.019; d3 = 0.1254; d4 = 0.4318;
L1 = [];
L2 = [];
L3 = [];
L4 = [];
L5 = [];
L6 = [];
Puma_sim = SerialLink([L1 L2 L3 L4 L5 L6],'name','Puma 560')
time = [0 1 2];
t1 = [0 -pi/4 -pi/2]; % We are only moving joint 1 for simplicity
t2 = [0 0 0]; t3 = [0 0 0]; t4 = [0 0 0]; t5 = [0 0 0]; t6 = [0 0 0];

for i = 1:3
    [] % Fill this out
end