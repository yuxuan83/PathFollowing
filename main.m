%% Paramenters
dt = 0.1; % sampling period [s]
T = 6; % total time span [s]

%% 1. generate reference trajectory
% given waypoints solve open loop characteristic and corresponding input
% function (steering angle and acceleration)

%% 2. interpolation reference trajectroy
% vq = interp1(x, v, xq) 

%% 3. linearization about the reference trajectory
% x(i+1) = Ai * x(i) + Bi * u(i)

%% 4. MPC loop
% define MPC paramenters and constraints
% initial condition X0 = [x0, y0, psi0, v0]

% for
%   compute equality constraint
%   compute inequality constraint
%   solve qp
%   simulate output using ode45 function
%   record actual state x
% end
