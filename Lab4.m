% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 0.8;
Joystick_gain_y = -0.8;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]
V_s_0 = 8.5;
K_f = -(m_c*g*l_c-2*m_p*g*l_h)/(V_s_0*l_h);
J_e = m_c*l_c^2 +2*m_p*l_h^2;
J_l = m_c*(l_c)^2 +2*m_p*(l_h^2 +l_p^2);
L_1 = l_p*K_f;
L_2 = m_c*g*l_c-2*m_p*g*l_h;
L_3 = K_f*l_h;
L_4 = K_f*l_h;
K_1 = L_1/(2*m_p*(l_p)^2);
K_2 = L_3/J_e;
K_3 = -L_2/J_l;


PORT = 5;

%SYSTEM
A = [0 1 0 0 0; 
    0 0 0 0 0;
    0 0 0 0 0;
    1 0 0 0 0;
    0 0 1 0 0];

B = [0 0;
    0 K_1;
    K_2 0;
    0 0;
    0 0];
%LQR
Q=[45 0 0 0 0;
    0 40 0 0 0;
    0 0 100 0 0;
    0 0 0 12 0;
    0 0 0 0 6];

R=[0.7 0;
    0 0.7];

K = lqr(A,B,Q,R);

F = [K(1,1) K(1,3); 
    K(2,1) K(2,3)];

%Luenberger Estimator
p=[-80+10i -80 -80 -10 -80-10i]/2;

A_e = [0 1 0 0 0;
    0 0 0 0 0;
    0 0 0 1 0;
    0 0 0 0 0;
    K_3 0 0 0 0];

B_e =[0 0;
    0 K_1; 
    0 0;
    K_2 0;
    0 0];

C_e=[1 0 0 0 0;
    0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0;
    0 0 0 0 1];

L_e = place(A_e', C_e', p)';

IMU_offset = [0.055;
              0.002;
              0.05;
              0.04;
              0.02];
              

%Kalman
Ts = 0.002;

A_c = [0 1 0 0 0 0; 
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    1 0 0 0 0 0;
    0 0 0 0 0 1;
    0 0 1 0 0 0];

B_c = [0 0;
    0 K_1;
    K_2 0;
    0 0;
    0 0;
    0 0];

C_c = [1 0 0 0 0 0; 
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 1];

D_c = [0 0; 
    0 0;
    0 0;
    0 0;
    0 0];
sysc = ss(A_c, B_c, C_c, D_c);
sysd= c2d(sysc, Ts);

A_d = sysd.A;
B_d = sysd.B;
C_d = sysd.C;

% Tuned 
Q_d =diag([0.00005 0.00001 0.0001 0.0002 0.0001 0.0001]);

%Experimentation
%Q_d =diag([1000000 1000000 1000000 1000000 1000000 1000000]);

load('Ground.mat')

load('linearization.mat')


ground = transpose(gr(2:6,:));
linearization = transpose(lin(2:6,:));


R_d = cov(linearization);

ground_cov = cov(ground);

%PLOTS
% figure(1)
% a1 = plot(gr(1,:),gr(2,:), 'k','LineWidth', 2);
% title('Pitch, ground', 'FontSize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(2)
% a2 = plot(gr(1,:),gr(3,:), 'k','LineWidth', 2);
% title('Pitch rate, ground', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(3)
% a3 = plot(gr(1,:),gr(4,:), 'k','LineWidth', 2);
% title('Elevation, ground', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(4)
% a4 = plot(gr(1,:),gr(5,:), 'k','LineWidth', 2);
% title('Elevation rate, ground', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(5)
% a5 = plot(gr(1,:),gr(6,:), 'k','LineWidth', 2);
% title('Travel rate, ground', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(6)
% a6 = plot(lin(1,:),lin(2,:), 'k','LineWidth', 2);
% title('Pitch, linearization', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(7)
% a7 = plot(lin(1,:),lin(3,:), 'k','LineWidth', 2);
% title('Pitch rate, linearization', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(8)
% a8 = plot(lin(1,:),lin(4,:), 'k','LineWidth', 2);
% title('Elevation, linearization', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% 
% figure(9)
% a9 = plot(lin(1,:),lin(5,:), 'k','LineWidth', 2);
% title('Elevation rate, linearization', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 
% figure(10)
% a10 = plot(lin(1,:),lin(6,:), 'k','LineWidth', 2);
% title('Travel rate, linearization', 'Fontsize', 18)
% xlabel('t[s]', 'FontSize', 16)
% ylabel('[rad]', 'FontSize', 16)
% 


