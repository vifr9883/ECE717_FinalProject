clear
clc
close all

%% User parameters
linearization_method = 1; % 1=F-LTI, 2=E-LTI, 3=D-LTI
control_method = 'lyap'; % 'lyap' or 'LQR'
trajectory_type = 'mild'; % 'mild' or 'aggr'
show_plot = true;
save_figures = false;


%% Get Trajectory
if strcmp(trajectory_type,'mild')
    D = readmatrix("traj_vcp.csv"); % Read in data
elseif strcmp(trajectory_type,'aggr')
    D = readmatrix("traj_figW.csv"); % Read in data
end
[t, x_r, u_r] = to_state(D); % Convert to state/input vectors
% To quaternion space
xi_r = x_r(4:6,:);
q_r = eul2quat(x_r(4:6,:)',"XYZ")';
x_r = [x_r(1:3,:);q_r;x_r(7:9,:)];    
% Initial condition
x_0 = x_r(:,1);

%% Get Controller Parameters / Linearization
quatquad_analysis;
if strcmp(control_method,'lyap')
    K_f = K_f_lyap;
    K_e = K_e_lyap;
    K_d1 = K_d1_lyap;
    K_d2 = K_d2_lyap;
elseif strcmp(lower(control_method),'lqr')
    K_f = K_f_lqr;
    K_e = K_e_lqr;
    K_d1 = K_d1_lqr;
    K_d2 = K_d2_lqr;
end

%% Setup Simulation
u_eq = [9.81;0;0;0];
u_r_sim.time = t;
u_r_sim.signals.values = u_r';
u_r_sim.signals.dimensions = 4;
x_r_sim.time = t;
x_r_sim.signals.values = x_r';
x_r_sim.signals.dimensions = 10;

tsim = t(end);
Ts = t(2)-t(1);

%% Simulate
out = sim('quatquad_sim.slx');
x = out.x';
u = out.u';

% Recover euler
xi = quat2eul(x(4:7,:)',"XYZ")';


% Show IAE (POS)
IAE_att = compute_IAE(x(1:3,:), x_r(1:3,:), Ts);
disp(strcat('Position IAE: ',string(IAE_att)))

% Show IAE (ATT)
IAE_att = compute_IAE(x(4:7,:), x_r(4:7,:), Ts);
disp(strcat('Attitude IAE: ',string(IAE_att)))


%% Plot
if linearization_method == 1
    if strcmp('lyap',control_method)
        col = 'b';
    else
        col = 'c';
    end
elseif linearization_method == 2
    if strcmp('lyap',control_method)
        col = '--m';
    else
        col = '--r';
    end
else
    col = 'g';
end

if show_plot
    if ~ishandle(1)
        plot_3D(1,x_r(1:3,:),'--k',[],[],[],["$x$ [m]","$y$ [m]", "$z$ [m]"])
        plot_tiled(2,x_r(1:3,:),'--k',t,["x [m]","y [m]", "z [m]"],[3,1])
        plot_tiled(3,rad2deg(xi_r),'--k',t,["$\phi$ [deg]","$\theta$ [deg]", "$\psi$ [deg]"],[3,1])
        plot_tiled(4,[u_r(1,:); rad2deg(u_r(2:4,:))],'--k',t,["$T$ [m/$s^2$]","$p$ [deg/s]", "$q$ [deg/s]","$r$ [deg/s]"],[2,2])
%         plot_tiled(5,x_r(8:10,:),col,t,["$\dot{x}$ [m/s]","$\dot{y}$ [m/s]", "$\dot{z}$ [m/s]"],[3,1])
        plot_tiled(6,x_r(4:7,:),'--k',t,["$q_0$ [-]","$q_1$ [-]", "$q_2$ [-]", "$q_3$ [-]"],[2,2])
    end
    figure(1)
    hold on
    plot3(x(1,:),x(2,:),x(3,:),col,'LineWidth',1.5)
    if strcmp(trajectory_type,'mild')
        view(34,28)
    else
        view(-100,20)
    end
    hold off
    plot_tiled(2,x(1:3,:),col,t,[],[])
    plot_tiled(3,rad2deg(xi),col,t,[],[])
    plot_tiled(4,[u(1,:);rad2deg(u(2:4,:))],col,t,[],[])
%     plot_tiled(5,x(8:10,:),col,t,[],[])
    plot_tiled(6,x(4:7,:),col,t,[],[])
end

if save_figures && linearization_method == 2
    save_figs(control_method,trajectory_type);
end

%% Functions
function [t, x_r,u_r] = to_state(D)
  % D is data
  g = 9.81;
  % Get time vector
  t = D(:,1);
  % Initialize state and input
  x_r = zeros(9,length(t)); % [x y z phi theta psi dx dy dz]
  u_r = zeros(4,length(t)); % [T p q r]
  % Retrieve flat outputs
  r = D(:,2:4)';
  r_dot = D(:,6:8)';
  r_ddot = D(:,10:12)';
  r_dddot = D(:,13:15)';
  % Differential Flatness
  x_r(1:3,:) = r; % x y z
  x_r(7:9,:) = r_dot; % dx dy dz
  T = r_ddot + g*[0;0;1];
  u_r(1,:) = vecnorm(T); % T
  z_B = T./u_r(1,:);
  y_C = [-sin(0);cos(0);0]*ones(1,length(t)); % Assume psi(t) = 0
  x_B = cross(y_C,z_B)./vecnorm(cross(y_C,z_B));
  y_B = cross(z_B,x_B);
  x_r(5,:) = -asin(x_B(3,:)); % theta
  x_r(4,:) = asin(y_B(3,:)./cos(x_r(5,:))); % phi
  h_Omega = (1./u_r(1,:)).*(r_dddot - dot(z_B,r_dddot).*z_B);
  u_r(2,:) = -dot(y_B,h_Omega); % p
  u_r(3,:) = dot(x_B,h_Omega); % q
  u_r(4,:) = dot(z_B,zeros(1,length(t)).*[0;0;1]); % r ( Assume psi_dot(t) = 0 )  
end
function [] = save_figs(control_method, trajectory_type)
    figure(1)
    if strcmp(control_method,'lyap')
        legend("Trajectory","LYAP-F","LYAP-E","Location","best")
    else
        legend("Trajectory","LQR-F","LQR-E","Location","best")
    end
    if strcmp(trajectory_type,'mild')
        traj = 'vcp';
    else
        traj = 'figW';
    end
    % Export graphics as eps
    prefix = strcat(traj,"_",lower(control_method),"_");
    sufix = ".eps";
    exportgraphics(figure(1),strcat(prefix,"3D",sufix),'Resolution',900);
    exportgraphics(figure(2),strcat(prefix,"2D",sufix),'Resolution',900);
    exportgraphics(figure(3),strcat(prefix,"eul",sufix),'Resolution',900);
    exportgraphics(figure(4),strcat(prefix,"inp",sufix),'Resolution',900);
    exportgraphics(figure(6),strcat(prefix,"q",sufix),'Resolution',900);
end
