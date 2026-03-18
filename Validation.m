clear
clc
close all


% Build kinematics first
l1 = 0.3;
l2 = 0.3;
l3 = 0.3;
L1(1) = Link([0 0 0 0], 'modified');
L1(2) = Link([0 0 l1 pi/2], 'modified');
L1(3) = Link([0 0 l2 0], 'modified');
L1(4) = Link([0 0 l3 0], 'modified');
Robot = SerialLink(L1, 'name', '3D-1-RRR');

% Define the input for Jacobian funciton
th = [0, 0, 0, 0]; % Define joint angles for the robot

T{1} = Robot.A([1], th).T;
T{2} = Robot.A([2], th).T;
T{3} = Robot.A([3], th).T;
T{4} = Robot.A([4], th).T;

method = "explicit";
joint = 4;
joint_type = [1 1 1 1];
joint_velocity = [1 1 1 0]';

% Validation
J = calculateJacobian(T, joint, joint_type, method);
J_tb = Robot.jacob0(th); 
vel = J * joint_velocity
vel_tb = J_tb*joint_velocity

if norm(vel - vel_tb) <= 0.0001
    disp("The results are equal!")
else
    disp("Something went wrong!")
end