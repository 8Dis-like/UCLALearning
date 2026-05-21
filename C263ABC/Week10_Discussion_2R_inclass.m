clear all; close all; clc

syms l1 l2 t1 t2 t3 m1 m2 dt1 dt2 ddt1 ddt2 g
assume([l1 l2 t1 t2 t3 m1 m2 dt1 dt2 ddt1 ddt2 g],'real')
L(1) = Link('revolute','d', 0, 'a', 0, 'alpha', 0 ,'modified');
L(2) = Link('revolute','d', 0, 'a', l1, 'alpha', 0 ,'modified');
L(3) = Link('revolute','d', 0, 'a', l2, 'alpha', 0 ,'modified');

RR1 = SerialLink(L, 'name', '2D-1-RR');
th = [t1 t2 0];

T0_1 = RR1.A([1], th);
T1_2 = RR1.A([2], th);
T2_T = RR1.A([3], th);
T0_T = RR1.A([1 2 3], th);
T0_2 = RR1.A([1 2],th);

%%
[R0_1, P0_1] = tr2rt(T0_1); R1_0 = transpose(R0_1);
[R1_2, P1_2] = tr2rt(T1_2); R2_1 = transpose(R1_2);
[R0_2, P0_2] = tr2rt(T0_2); R2_0 = transpose(R0_2);
[R2_T, P2_T] = tr2rt(T2_T); RT_2 = transpose(R2_T);
[R0_T, P0_T] = tr2rt(T0_T);

%% Define center of mass positions and moment of inertia
PC1 = [l1/2; 0 ; 0];
PC2 = [l2/2; 0 ; 0];

IC1 = (1/12) * m1 * l1^2 * [0 0 0; 0 1 0; 0 0 1];
IC2 = (1/12) * m2 * l2^2 * [0 0 0; 0 1 0; 0 0 1];

I1 = []; % fill this out
I2 = []; % fill this out
p0c1 = []; % fill this out; 
p0c1 = p0c1(1:3,1);
p0c2 = []; % fill this out 
p0c2 = p0c2(1:3,1);
%% Link Jacobian Matrix
Jv1 = sym(zeros(3,2)); 
Jw1 = sym(zeros(3,2));
Jv2 = sym(zeros(3,2)); 
Jw2 = sym(zeros(3,2));

Jv1(1:3,1) = []; % fill this out 
Jw1(1:3,1) = []; % fill this out 

Jv2(1:3,1) = []; % fill this out 
Jw2(1:3,1) = []; % fill this out 
Jv2(1:3,2) = []; % fill this out 
Jw2(1:3,2) = []; % fill this out 

%% Manipulator inertia matrix
M = []; % fill this out 

%% Velocity coupling vector
dq = [dt1 dt2];
V1 = sym(0); V2 = sym(0);
for j = 1:2
    for k = 1:2
        V1 = []; % fill this out 
        V2 = []; % fill this out 
    end
end


for j = 1:2
    for k = 1:2
        
    end
end


V1 = simplify(V1);
V2 = simplify(V2);
%% Gravitational vector(Assume g = [0 0 -g])
G1 = sym(0); G2 = sym(0);
m = [m1,m2];
Jv = {Jv1 Jv2};
for j = 1:2
    G1 = []; % fill this out 
    G2 = []; % fill this out 
end


simplify(G1)
simplify(G2)
%% Conclusion: M * qdd + V + G = Q
V = [V1; V2]
G = [G1; G2]