clear
close all
clc

%% Paramenters
dt = 0.05; % sampling period [s]
sim_dt = 0.005; % simulation time step [s]

%% 1. Load reference trajectory
load('ref_traj_3.mat');
t_traj = (0:(length(X_ref)-1)) * dt;

%% 2. MPC loop
% Define MPC paramenters and constraints
Np = 20; % finite time step
Q = blkdiag(2, 2, 1, 1);
R = blkdiag(1, 0.1);
Q_blk = kron(eye(Np+1), Q);
R_blk = kron(eye(Np), R);
H_blk = blkdiag(Q_blk, R_blk);

% Initial condition x = [x0; y0; psi0; v0]
x0 = [-1.4; 1; 0.5; 11.6];

X_actual = [];
U_actual = [];

% computation time in each iteration
com_time = [];

% Iterations
for i = 1:length(t_traj)
    % Start tick to record performance
    tic
    if i + Np > length(t_traj)
        % Case for the tail of trajectory where i + Np > length(t_traj)      
        N = length(t_traj) - i + 1;
        
        % Construct a temporary weight matrix
        Q_blk_temp = kron(eye(N+1), Q);
        R_blk_temp = kron(eye(N), R);
        H_blk_temp = blkdiag(Q_blk_temp, R_blk_temp);
        
        % Reference trajectory
        x_ref = X_ref(:,i:i+N-1);
        u_ref = U_ref(:,i:i+N-1);
        
        % Construct equality constraint matrix
        [Aeq, beq] = eq_constraint(x_ref, u_ref, (x0-x_ref(:,1)), dt, N);

        % Inequality constraint
        [lb, ub] = lu_bound(u_ref, N);

        % Solve qp
        D_star = cplexqp(H_blk_temp, zeros((6*N+4),1), [], [], Aeq, beq, lb, ub);

        % Stop tick
        time_elapse = toc;

        % exract the first optimal control input
        du_star = D_star(4*(N+1)+1:4*(N+1)+2,1);
    else
        % Normal case
        
        % Reference trajectory
        x_ref = X_ref(:,i:i+Np-1);
        u_ref = U_ref(:,i:i+Np-1);

        % Construct equality constraint matrix
        [Aeq, beq] = eq_constraint(x_ref, u_ref, (x0-x_ref(:,1)), dt, Np);

        % Inequality constraint
        [lb, ub] = lu_bound(u_ref, Np);

        % Solve qp
        D_star = cplexqp(H_blk, zeros((6*Np+4),1), [], [], Aeq, beq, lb, ub);

        % exract the first optimal control input
        du_star = D_star(4*(Np+1)+1:4*(Np+1)+2,1);
    end
    % Stop tick
    time_elapse = toc;
    
    % Vehicle control input
    u_star = du_star + u_ref(:,1);
    
    % Simulate output using ode45 function
    [~, x] = ode45(@(t,x) vehicle_dynamics(x, u_star), 0:sim_dt:dt, x0);
    
    % Record actual state x and output
    x0 = x(end,:)';
    X_actual = [X_actual, x(2:end,:)'];
    U_actual = [U_actual, u_star];

    % Record computational time
    com_time = [com_time, time_elapse];
end

% Prost process of data
com_time_avg = mean(com_time);
t_actual = (0:(length(X_actual)-1)) * sim_dt;

%% Plot results
% Plot path: x versus y
figure(1)
plot(X_ref(1,:), X_ref(2,:), '--', X_actual(1,:), X_actual(2,:), 'LineWidth', 1.2)
axis equal
grid on
title('Path')
%xlim([-0.5, max(X_ref(1,:))+1])
% xlim([0,40]);
xlabel('x')
ylabel('y')

% Plot states versus time
figure(2)

% x
subplot(4,1,1) 
plot(t_traj, X_ref(1,:), '--', t_actual, X_actual(1,:), 'LineWidth', 1.2)
grid on
xlabel('time (s)')
ylabel('x (m)')
xlim([-inf inf])
% y
subplot(4,1,2)
plot(t_traj, X_ref(2,:), '--', t_actual, X_actual(2,:), 'LineWidth', 1.2)
grid on
xlabel('time (s)')
ylabel('y (m)')
xlim([-inf inf])
% psi
subplot(4,1,3)
plot(t_traj, X_ref(3,:), '--', t_actual, X_actual(3,:), 'LineWidth', 1.2)
grid on
xlabel('time (s)')
ylabel('\psi (rad)')
xlim([-inf inf])
% u
subplot(4,1,4)
plot(t_traj, X_ref(4,:), '--', t_actual, X_actual(4,:), 'LineWidth', 1.2)
grid on
xlabel('time (s)')
ylabel('u (m/s)')
xlim([-inf inf])

% Plot input versus time
figure(3)
subplot(2,1,1)
plot(t_traj, U_ref(1,:), '--', t_traj, U_actual(1,:), 'LineWidth', 1.2);
xlabel('time (s)')
ylabel('\delta (rad)')
ylim([-1 1]);
grid on
subplot(2,1,2)
plot(t_traj, U_ref(2,:), '--', t_traj, U_actual(2,:), 'LineWidth', 1.2);
xlabel('time (s)')
ylabel('a (m/s^2)')
ylim([-1.6 1.6]);
grid on

% Plot computational time in each iterations
figure(4)
title('Computational Time')
plot(1:length(com_time), com_time, 'x', 1:length(com_time), dt*ones(1,length(t_traj)), '--', 'LineWidth', 1.2)
text(1,0.06, sprintf('average time: %.4f', com_time_avg))
grid on
title('computation time')
xlabel('time (s)')
ylabel('computation time')
ylim([0 0.08])
xlim([-inf inf])


%% Function for constructing equality constraints and inequality constraints
function [Aeq, beq] = eq_constraint(x_ref, u_ref, x0, dt, N)

    beq = zeros(4*(N+1),1);
    beq(1:4, 1) = x0;

    A_cell = cell(N,1);
    B_cell = cell(N,1);

    for i = 1:N
        [A_cell{i}, B_cell{i}] = ltv_mdl(x_ref(:,i), u_ref(:,i), dt);
    end

    Aeq_cell = cell(N+1, 2*N+1);
    Aeq_cell(:,1:N+1) = {zeros(4,4)};
    Aeq_cell(:,N+2:end) = {zeros(4,2)};
    Aeq_cell{1,1} = eye(4);
    for j = 2:N+1
        Aeq_cell{j,j-1} = A_cell{j-1};
        Aeq_cell{j,j} = -eye(4);
        Aeq_cell{j,j+N} = B_cell{j-1};
    end

    Aeq = cell2mat(Aeq_cell);

end

function [lb, ub] = lu_bound(u_ref, N)
    % Input and state contraints
    delta_upper = 37 * (pi/180); % [rad]
    delta_lower = -37 * (pi/180); % [rad]
    acc_upper = 1; % [m/s^2]
    acc_lower = -1.5; % [m/s^2]
    x_upper = 3; % [m]
    x_lower = -3; % [m]
    y_upper = 3; % [m]
    y_lower = -3; % [m]
    psi_upper = 90 * (pi/180); % [rad]
    psi_lower = -90 * (pi/180); % [rad]
    u_upper = 5; % [m/s]
    u_lower = -5; % [m/s]
    
    u_ref_v = reshape(u_ref, 2*N,1);
    
    x_ub = repmat([x_upper; y_upper; psi_upper; u_upper], (N+1), 1);
    x_lb = repmat([x_lower; y_lower; psi_lower; u_lower], (N+1), 1);
    u_ub = repmat([delta_upper; acc_upper], N, 1) - u_ref_v;
    u_lb = repmat([delta_lower; acc_lower], N, 1) - u_ref_v;
    
    ub = [x_ub; u_ub];
    lb = [x_lb; u_lb];
    
end