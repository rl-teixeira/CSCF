%CPCS
clear all, close all, clc

set(0,'defaultTextInterpreter','latex');
set(0,'defaultLegendInterpreter','latex');

%% 1.1.1
m= 0.1;                  %kg 
Cd= 0.3;                 %
A= 0.1;                  %m^2
ro= 1.225;               %kg.m^-3
g= 9.8065;               %m.s^-2
theta_max= 30*(pi/180);  %rad
theta_min= -30*(pi/180); %rad
d_r= 0.5;                %m
d_max= 5;                %m
d_min= 0.2;              %m
v0e= 3;                  %m/s
we= 0;                   %m/s

K= (ro*Cd*A)/(2*m);

%% 1.1.2

theta_e= atan((K*v0e^2)/g);

%% 1.1.3

A= [0 -1; 0 -2*K*v0e];
B= [0 g/cos(theta_e)^2]';
C= eye(2);
D= [0];
drone_ss= ss(A,B,C,D);

%% 1.1.4

Ts= 0.1; %s
%forward Euler aproximation
Ad= eye(2) + A*Ts;
Bd= B*Ts;
Cd= C;
Dd= D;

%exact discretizaton
drone_ss_d= c2d(drone_ss,Ts)

%% 1.1.5

[V,D]= eig(drone_ss_d.A);

Mc= [drone_ss_d.B drone_ss_d.A*drone_ss_d.B];
rank(Mc)
Mo= [drone_ss_d.C; drone_ss_d.C*drone_ss_d.A];
rank(Mo)


%% 1.1.6

t=0:Ts:10;
in_step= 0.1*heaviside(t);
out_c= lsim(drone_ss,in_step,t,[0 0]');
out_d= lsim(drone_ss_d,in_step,t,[0 0]');

figure (101)
subplot(2,1,1)
plot(out_d(:,1),'b')
hold on
plot(out_c(:,1),'r')
legend('discretized','continuous')
title('\textbf{$P_{10}$} - Position')
subplot(2,1,2)
plot(out_d(:,2),'b')
hold on
plot(out_c(:,2),'r')
legend('discretized','continuous')
title('\textbf{$V_1$} - Velocity')

figure (102)
plot(out_d(:,1),out_d(:,2))
title('Phase Plot')
xlabel('$P_{10}$')
ylabel('$V_1$')

%% 1.1.7

theta_e_2D= atan((K/(m*g))*v0e);
phi_e_2D= atan((K*cos(theta_e_2D)*v0e)/(m*g));

A_2D= [0 0 0 0; 0 0 0 0; -1 0 -2*K*v0e 0; 0 -1 0 -2*K*v0e]';
B_2D= [0 0; 0 0; 0 m*g*sec(theta_e_2D)^2; (m*g*tan(theta_e_2D)*tan(phi_e_2D))/(cos(phi_e_2D)) m*g*sec(theta_e_2D)^2/cos(phi_e_2D)];
C_2D= [1 0 0 0; 0 1 0 0];
D_2D=[0];

%% 1.1.8

drone_ss_2D= ss(A_2D,B_2D,C_2D,D_2D);
drone_ss_d_2D= c2d(drone_ss_2D,Ts);

in_step_2D= [in_step; in_step];
out_2D= lsim(drone_ss_d_2D,in_step_2D,t,[0 0 0 0]');
lsim(drone_ss_d_2D,in_step_2D,t,[0 0 0 0]')
figure(103)
plot(out_2D(:,1),out_2D(:,2))
xlabel('$P_{10_x}$'), ylabel('$P_{10_y}$')
title('\textbf{$P_{10}$} - Position')


%% saves

save drone_model.mat drone_ss_d drone_ss_d_2D Ts

