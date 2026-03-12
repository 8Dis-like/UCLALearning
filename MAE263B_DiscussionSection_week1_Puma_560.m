clear all; close all; clc
syms t1 t2 t3 t4 t5 t6 a2 a3 d3 d4 pi

L1 = Link('revolute','d', 0, 'a', 0,'alpha', 0, 'modified')
L2 = Link('revolute','d', 0, 'a', 0,'alpha', -pi/2, 'modified')
L3 = Link('revolute','d', d3, 'a', a2,'alpha', 0, 'modified')
L4 = Link('revolute','d', d4, 'a', a3,'alpha', -pi/2, 'modified')
L5 = Link('revolute','d', 0, 'a', 0,'alpha', pi/2, 'modified')
L6 = Link('revolute','d', 0, 'a', 0,'alpha', -pi/2, 'modified')

th = [t1 t2 t3 t4 t5 t6];
Puma560 = SerialLink([L1 L2 L3 L4 L5 L6],'name','Puma 560')

%%
format short
Puma_fk = simplify(Puma560.fkine(th))

%%
clear pi
th = [0 0 0 0 0 0];
a2 = 0.4318; a3 = 0.019; d3 = 0.1254; d4 = 0.4318;
L1 = Link('revolute','d', 0, 'a', 0,'alpha', 0, 'modified')
L2 = Link('revolute','d', 0, 'a', 0,'alpha', -pi/2, 'modified')
L3 = Link('revolute','d', d3, 'a', a2,'alpha', 0, 'modified')
L4 = Link('revolute','d', d4, 'a', a3,'alpha', -pi/2, 'modified')
L5 = Link('revolute','d', 0, 'a', 0,'alpha', pi/2, 'modified')
L6 = Link('revolute','d', 0, 'a', 0,'alpha', -pi/2, 'modified')
Puma_sim = SerialLink([L1 L2 L3 L4 L5 L6],'name','Puma 560')
%%
time = [0 5 10];
t1 = [0 -pi/4 -pi/2];
t2 = [0 0 0]; t3 = [0 0 0]; t4 = [0 0 0]; t5 = [0 0 0]; t6 = [0 0 0];

for i = 1:3
    th = [t1(i) t2(i) t3(i) t4(i) t5(i) t6(i)]
    Puma_sim.plot(th,'delay',0.5)
end
