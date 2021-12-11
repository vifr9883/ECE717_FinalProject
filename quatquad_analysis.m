% Analyze the properties of the
% Linearized Quaternion-based Quadcopter Model

% clear
% clc
% close all

g = 9.81;

%% Feedback Linearization
% Single System
A_f = zeros(10);
B_f = zeros(10,7);
A_f(1:3,4:6) = eye(3);
B_f(4:6,1:3) = eye(3);
B_f(7:9,4:6) = eye(3);
B_f(end,end) = 1;
C_f = ctrb(A_f,B_f);
rank(C_f);

% Double System
A_d1 = [zeros(3),eye(3);zeros(3),zeros(3)];
B_d1 = [zeros(3);eye(3)];
C_d1 = ctrb(A_d1,B_d1);
rank(C_d1);
A_d2 = zeros(4);
B_d2 = eye(4);
C_d2 = ctrb(A_d2,B_d2);
rank(C_d2);


%% Equilibrium Linearization
A_e = zeros(10,10);
B_e = zeros(10,4);
A_e(1:3,8:10) = eye(3);
M_A = 2*g*[0 0 1 0; 0 -1 0 0; 1 0 0 0];
M_B = 0.5*[0 0 0; eye(3)];
A_e(8:10,4:7) = M_A;
B_e(8:10,1) = [0;0;1];
B_e(4:7,2:4) = M_B;
C_e = ctrb(A_e,B_e);
[Abar_e,Bbar_e,Cbar_e,T_e,k] = ctrbf(A_e,B_e,eye(10));

% Find uncontrollable state
syms x y z q_0 q_1 q_2 q_3 dx dy dz real
x_vec = [x y z q_0 q_1 q_2 q_3 dx dy dz]';
z_vec = T_e*x_vec;

% Extract controllable matrices
A_e = Abar_e(2:end,2:end);
B_e = Bbar_e(2:end,:);
C_e = ctrb(A_e,B_e);
rank(C_e);

%% Lyapunov Controller design
% (F-LTI)
mu = 2;
W = lyap(-mu*eye(10)-A_f,B_f*B_f');
K_f_lyap = 1/2*B_f'/W; % Static feedback gain (F-LTI)
% (E-LTI)
W = lyap(-mu*eye(9)-A_e,B_e*B_e');
K_e_lyap = 1/2*B_e'/W; % Static feedback gain (E-LTI)
S_c = [zeros(9,1),eye(9)];
% (D-LTI)
W = lyap(-mu*eye(6)-A_d1,B_d1*B_d1');
K_d1_lyap = 1/2*B_d1'/W; % Static feedback gain (D1-LTI)
W = lyap(-mu*eye(4)-A_d2,B_d2*B_d2');
K_d2_lyap = 1/2*B_d2'/W; % Static feedback gain (D2-LTI)


%% LQR
% (F-LTI)
Q = diag([200,200,200,50,50,50,10,10,10,20]);
R = diag([0.1,0.1,0.1,0.01,0.01,0.01,0.05]);
K_f_lqr = lqr(A_f,B_f,Q,R); % Static feedback gain (F-LTI)
% (E-LTI)
Q = diag([100,100,100,50,50,10,10,10,50]);
R = diag([0.01,0.1,0.1,0.1]);
K_e_lqr = lqr(A_e,B_e,Q,R); % Static feedback gain (E-LTI)
% (D-LTI)
Q = diag([100,100,100,10,10,10]);
R = diag([0.1,0.1,0.1]);
K_d1_lqr = lqr(A_d1,B_d1,Q,R); % Static feedback gain (D1-LTI)
Q = diag([10,10,10,10]);
R = diag([0.1,0.1,0.1,0.1]);
K_d2_lqr = lqr(A_d2,B_d2,Q,R); % Static feedback gain (D2-LTI)

