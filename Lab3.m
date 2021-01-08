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

%ESTIMATOR
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
