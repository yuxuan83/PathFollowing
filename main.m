clear
close all
clc

%% Paramenters
dt = 0.05; % sampling period [s]
T_max = 6; % total time span [s]
T = 0:dt:T_max;

%% 1. load and interpolate reference trajectory
load('ref_traj.mat');

U_ref = interp1(0:0.05:T_max, [U(:,1),U]', T)';
Y_ref = interp1(0:0.05:T_max, Y', T)';

figure(1)
plot(Y_ref(1,:), Y_ref(2,:), '--', 'LineWidth', 1.2)
axis equal
xlim([-0.5, 6.5])

figure(2)
subplot(2,1,1)
plot(T, U_ref(1,:), '--', 'LineWidth', 1.2);
ylim([-1 1]);
grid on
subplot(2,1,2)
plot(T, U_ref(2,:), '--', 'LineWidth', 1.2);
ylim([-1 1]);
grid on

%% 2. MPC loop
% define MPC paramenters and constraints
n_pred = 10; % finite time step

Ndec=3*(n_pred+1)+2*n_pred; 

% initial condition X0 = [x0; y0; psi0; v0]
x0 = [0.25; 0.25; -1; 5];

% input and state contraints
delta_upper = 37 * (pi/180); % [rad]
delta_lower = -37 * (pi/180); % [rad]
acc_upper = 1.5; % [m/s^2]
acc_lower = -1.5; % [m/s^2]
y_upper = 1.6; % [m]
y_lower = -1.6; % [m]
x_upper = 3; % [m]
x_lower = -1; % [m]



% for loop
%   compute equality constraint with linearized model (trajectory)
%   compute inequality constraint
%   solve qp
%   simulate output using ode45 function
%   record actual state x
% end
