clear
close all
clc

%% Paramenters
dt = 0.05; % sampling period [s]

%% 1. Load reference trajectory
load('ref_traj_5.mat');
t_traj = 0:dt:(length(X_ref)-1)*dt;

%% 2. MPC loop
% Define MPC paramenters and constraints
Np = 10; % finite time step
Q = blkdiag(10, 10, 1, 1);
R = blkdiag(0.5,0.1);
Q_blk = kron(eye(Np+1), Q);
R_blk = kron(eye(Np), R);
H_blk = blkdiag(Q_blk, R_blk);

% Initial condition x = [x0; y0; psi0; v0], u = [delta0, acc0]
x0 = [0; 0; 1.5; 2.5];
X_actual = [];
U_actual = [];

% computation time in each iteration
com_time = [];

% Iterations
for i = 1:length(t_traj)-Np
    % Start tick to record performance
    tic
    
    x_ref = X_ref(:,i:i+Np-1);
    u_ref = U_ref(:,i:i+Np-1);
    
    % Construct equality constraint matrix
    [Aeq, beq] = eq_constraint(x_ref, u_ref, (x0-x_ref(:,1)), dt);
    
    % Inequality constraint
    [lb, ub] = ineq_constraint(u_ref, Np);
    
    % Solve qp
    D_star = cplexqp(H_blk, zeros((6*Np+4),1), [], [], Aeq, beq, lb, ub);
    
    % Stop tick
    time_elapse = toc;
    
    % exract the first optimal control input
    du_star = D_star(4*(Np+1)+1:4*(Np+1)+2,1);
    u_star = du_star + u_ref(:,1);
    
    % Simulate output using ode45 function
    [~, x] = ode45(@(t,x) vehicle_dynamics(x, u_star), 0:0.005:dt, x0);
    
    % Record actual state x and output
    x0 = x(end,:)';
    X_actual = [X_actual, x'];
    U_actual = [U_actual, u_star];

    % Record computation time
    com_time = [com_time, time_elapse];
end

% Prost process of data
com_time_avg = mean(com_time);
t_iter = 0:dt:(length(com_time)-1)*dt;

%% Plot results
figure(1)
title('Path')
plot(X_ref(1,:), X_ref(2,:), '--', X_actual(1,:), X_actual(2,:), 'LineWidth', 1.2)
axis equal
grid on
xlim([-0.5, max(X_ref(1,:))+1])
xlabel('x');
ylabel('y');

figure(2)
subplot(2,1,1)
plot(t_traj, U_ref(1,:), '--', t_iter, U_actual(1,:), 'LineWidth', 1.2);
title('steering angle')
xlabel('time (s)')
ylabel('delta (rad)')
ylim([-1 1]);
grid on
subplot(2,1,2)
plot(t_traj, U_ref(2,:), '--', t_iter, U_actual(2,:), 'LineWidth', 1.2);
title('acceleration')
xlabel('time (s)')
ylabel('a (m/s^2)')
ylim([-1.6 1.6]);
grid on

figure(3)
plot(t_iter, com_time, t_iter, dt*ones(1,length(t_iter)), '--', 'LineWidth', 1.2)
str = sprintf('average time: %.4f', com_time_avg);
text(1,0.06, str)
grid on
title('computation time')
xlabel('time (s)')
ylabel('computation time')
ylim([0 0.08])
xlim([-inf inf])


%% Function for constructing equality constraints and inequality constraints
function [Aeq, beq] = eq_constraint(x_ref, u_ref, x0, dt)

    Np = length(x_ref);

    beq = zeros(4*(Np+1),1);
    beq(1:4, 1) = x0;

    A_cell = cell(Np,1);
    B_cell = cell(Np,1);

    for i = 1:Np
        [A_cell{i}, B_cell{i}] = ltv_mdl(x_ref(:,i), u_ref(:,i), dt);
    end

    Aeq_cell = cell(Np+1, 2*Np+1);
    Aeq_cell(:,1:Np+1) = {zeros(4,4)};
    Aeq_cell(:,Np+2:end) = {zeros(4,2)};
    Aeq_cell{1,1} = eye(4);
    for j = 2:Np+1
        Aeq_cell{j,j-1} = A_cell{j-1};
        Aeq_cell{j,j} = -eye(4);
        Aeq_cell{j,j+Np} = B_cell{j-1};
    end

    Aeq = cell2mat(Aeq_cell);

end

function [lb, ub] = ineq_constraint(u_ref, Np)
    % Input and state contraints
    delta_upper = 37 * (pi/180); % [rad]
    delta_lower = -37 * (pi/180); % [rad]
    acc_upper = 1.5; % [m/s^2]
    acc_lower = -1.5; % [m/s^2]
    x_upper = 2; % [m]
    x_lower = -2; % [m]
    y_upper = 2; % [m]
    y_lower = -2; % [m]
    psi_upper = 90 * (pi/180); % [rad]
    psi_lower = -90 * (pi/180); % [rad]
    u_upper = 5; % [m/s]
    u_lower = -5; % [m/s]
    
    u_ref_v = reshape(u_ref, 2*Np,1);
    
    x_ub = repmat([x_upper; y_upper; psi_upper; u_upper], (Np+1), 1);
    x_lb = repmat([x_lower; y_lower; psi_lower; u_lower], (Np+1), 1);
    u_ub = repmat([delta_upper; acc_upper], Np, 1) - u_ref_v;
    u_lb = repmat([delta_lower; acc_lower], Np, 1) - u_ref_v;
    
    ub = [x_ub; u_ub];
    lb = [x_lb; u_lb];
    
end