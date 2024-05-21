disp('Cartesian Manipulator')
syms a1 a2 a3 a4 d1 d2 d3

%% Link Lengths
a1 = 5;
a2 = 10;
a3 = 5;
a4 = 10;

%% Joint Variables
d1 = 8;
d2 = 8;
d3 = 8;

%% D-H Parameters [theta, d, r, alpha, offset]
% if prismatic joint: theta, d = 0, offset = 1, after offset put the value of d
% if revolute joint: theta = 0, after offset put the value of theta

H0_1 = Link([0,0,0,3*pi/2,1,a1]);
H0_1.qlim = [0 0];

H1_2 = Link([3*pi/2,0,0,3*pi/2,1,a2]);
H1_2.qlim = [0 d1];

H2_3 = Link([pi/2,0,0,3*pi/2,1,a3]);
H2_3.qlim = [0 d2];

H3_4 = Link([0,0,0,0,1,a4]);
H3_4.qlim = [0 d3];

Cart = SerialLink([H0_1 H1_2 H2_3 H3_4], 'name', 'Cart')
Cart.plot([0 0 0 0], 'workspace',[-30 30 -30 30 -30 30]) %plot at Origin position
figure(1)
Cart.teach

%% Path and Trajectory

t = 0:0.5:2

q0 = ([0 0 0 0])    
q1 = ([0 3 3 3])
q2 = ([0 3 3 7])
q3 = ([0 9 3 7])
q4 = ([0 9 9 7])
q5 = ([0 3 9 7])
q6 = ([0 3 3 7])
q7 = ([0 3 3 3])

%trajectory
Traj1 = jtraj(q0,q1,t)
Traj2 = jtraj(q1,q2,t)
Traj3 = jtraj(q2,q3,t)
Traj4 = jtraj(q3,q4,t)
Traj5 = jtraj(q4,q5,t)
Traj6 = jtraj(q5,q6,t)
Traj7 = jtraj(q6,q7,t)

figure(2)
plot(Cart,Traj1)
plot(Cart,Traj2)
plot(Cart,Traj3)
plot(Cart,Traj4)
plot(Cart,Traj5)
plot(Cart,Traj6)
plot(Cart,Traj7)
