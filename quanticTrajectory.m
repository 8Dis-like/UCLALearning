function [pos, vel, acc, jerk] = quanticTrajectory(t0, tf, q0, qf, v0, vf, a0, af, N)
% cubicTrajectory:
%   Generates a cubic polynomial trajectory in N points
%   satisfying boundary conditions: position & velocity at start and end.
%
% Rationale:
%   A third-order polynomial can satisfy 4 boundary conditions
%   (q(t0)=q0, q(tf)=qf, q'(t0)=v0, q'(tf)=vf).
%
% Inputs:
%   t0 - initial time
%   tf - final time
%   q0 - initial position
%   qf - final position
%   v0 - initial velocity
%   vf - final velocity
%   a0 - initial acceleration
%   af - final acceleration
%   N  - number of points
%
% Outputs:
%   pos - [1 x N] positions
%   vel - [1 x N] velocities
%   acc - [1 x N] accelerations
%   jerk   - [1 x N] jerk
% Create time vector
T = linspace(t0, tf, N);

% Solve for quantic polynomial coefficients a0..a3
% Polynomial: q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4* t^4 + c5 * t^5
% Boundary conditions lead to system:
%   [1    t0    t0^2    t0^3   t0^4   t0^5] [c0] =  [ q0 ]
%   [1    tf    tf^2    tf^3   tf^4   tf^5] [c1] = [ qf ]
%   [0    1    2*t0    3*t0^2    4*t0^3    5*t0^4] [c2] =  [ v0 ]
%   [0    1    2*tf    3*tf^2    4*tf^3    5*tf^4][c3] =  [ vf ]
%   [0    0    2    6*t0    12*t0^2    20*t0^3] [c4] =  [ a0 ]
%   [0    0    2    6*tf    12*tf^2    20*tf^3][c5]  = [ af ]

A = [ 1,    t0,    t0^2,    t0^3,   t0^4,   t0^5;
      1,    tf,    tf^2,    tf^3,   tf^4,   tf^5;
      0,    1,    2*t0,    3*t0^2,    4*t0^3,    5*t0^4;
      0,    1,    2*tf,    3*tf^2,    4*tf^3,    5*tf^4;
      0,    0,    2,    6*t0,    12*t0^2,    20*t0^3;
      0,    0,    2,    6*tf,    12*tf^2,    20*tf^3];
y = [q0; qf; v0; vf; a0; af];
% y = Ax
x = A \ y;   % [c0; c1; c2; c3; c4; c5]
% "\" is a smarter operator to find out solution for the equation y=Ax
% where x is the unknown.
% x = inv(A)*y;
c0 = x(1); c1 = x(2); c2 = x(3); c3 = x(4); c4 = x(5); c5 = x(6);

% Pre-allocate
pos = zeros(size(T));
vel = zeros(size(T));
acc = zeros(size(T));
jerk = zeros(size(T));
for i = 1:N
    t = T(i);
    pos(i) = c0 + c1*t + c2*t^2 + c3*t^3 + c4* t^4 + c5 * t^5;
    vel(i) = c1 + 2* c2*t + 3*c3*t^2 + 4*c4* t^3 + 5*c5 * t^4;
    acc(i) = 2* c2 + 6*c3*t + 12*c4* t^2 + 20*c5 * t^3;
    jerk(i) = 6*c3 + 24*c4* t + 60*c5 * t^2;
end

end