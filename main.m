clear
close all
clc

%% Paramenters
dt = 0.05; % sampling period [s]
T_max = 6; % total time span [s]
T = 0:dt:T_max;

%% 1. Load and interpolate reference trajectory
load('ref_traj.mat');

U_ref = interp1(0:0.05:T_max, [U(:,1),U]', T)';
X_ref = interp1(0:0.05:T_max, X', T)';

figure(1)
plot(X_ref(1,:), X_ref(2,:), '--', 'LineWidth', 1.2)
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
% Define MPC paramenters and constraints
Np = 10; % finite time step
Q = 10*eye(4);
R = 0.2*eye(2);
Q_blk = kron(eye(Np+1), Q);
R_blk = kron(eye(Np), R);

% Input and state contraints
delta_upper = 37 * (pi/180); % [rad]
delta_lower = -37 * (pi/180); % [rad]
acc_upper = 1.5; % [m/s^2]
acc_lower = -1.5; % [m/s^2]
y_upper = 1.6; % [m]
y_lower = -1.6; % [m]
x_upper = 3; % [m]
x_lower = -1; % [m]

% Initial condition x = [x0; y0; psi0; v0], u = [delta0, acc0]
x = [0.25; 0.25; -1; 0];
u = [0; 0];

% Iterations
for i = 1:1
    
    % Construct state transition matrix
    x_ref = X_ref(:,i:i+Np-1);
    u_ref = U_ref(:,i:i+Np-1);
    
    A_cell = cell(Np,1);
    B_cell = cell(Np,1);
    for j = 1:Np
        [A_cell{j}, B_cell{j}] = ltv_mdl(x_ref(:,j), u_ref(:,j), dt);
    end
    
    F_cell = cell(Np+1, 1);
    Phi_cell = cell(Np+1, Np);
    for j = 1:Np+1
        F_cell{j} = ones(4,4);
        for k = 1:Np
            if k < j
                Phi_cell{j, k} = ones(4,2);
            else
                Phi_cell{j, k} = zeros(4,2);
            end
            
        end
    end
    F = cell2mat(F_cell);
    Phi = cell2mat(Phi_cell);
    
    % Inequality constraint
    
    
%     H = Phi'*Q_blk*Phi + R_blk;
    % Solve qp
%     dU = cplexqp(H, zeros(4*(Np+1), 1), A, b);
    % Simulate output using ode45 function
    
    % Record actual state x and output 

end

%% Plot results

