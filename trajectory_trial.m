dt = 0.05; % time step (sec)

T = 5; % total time (sec)
v_x = 10; % [m/s]
dx = v_x*dt;

x_f_s = T*v_x;
x_r_s = 0:dx:x_f_s;
y_r_s = zeros(1,length(x_r_s));


%% Generate a straight trajectory
U_ref = zeros(2,101);
X_ref = [x_r_s; y_r_s;zeros(1,101);v_x*ones(1,101)];
figure(1);
savefile = 'ref_traj_1.mat';
save(savefile, 'U_ref', 'X_ref');
plot(x_r_s, y_r_s, '--', 'Linewidth', 1)
xlabel('X (m)')
ylabel('Y (m)')
grid on
axis equal

% %% Generate sinusoidal trajectory 
% x_f = T*v_x;
% x_r = 0:dx:x_f;
% y_r = 2*sin(x_r*(2*pi/(v_x*T)));
% psi_r = 2*(2*pi/(v_x*T))*cos(x_r*(2*pi/(v_x*T)));
% u_r = v_x * ones(1,101);
% U_ref = [zeros(1,101);zeros(1, 101)];
% X_ref = [x_r;y_r;psi_r;u_r];
% savefile = 'ref_traj_2.mat';
% save(savefile, 'U_ref', 'X_ref');
% figure(2)
% plot(x_r, y_r, '--', 'Linewidth', 1)
% ylim([-3, 3])
% xlabel('X (m)')
% ylabel('Y (m)')
% grid on
% axis equal
% 
% %% Generate turning trajectory
% R = 20; % [m]
% u_r = 12; % [m/s]
% theta = 0:1.2/40:pi/2 -1.2/40;
% x = [zeros(1,33),R - R*cos(theta), R:R/33:2*R - R/33];
% y = [0:R/33:R-R/33, R + R*sin(theta), 2*R*ones(1,33)];
% U_ref = [zeros(1,118);zeros(1,118)];
% X_ref = [x;y;pi/2*ones(1,33), pi/2 - theta, zeros(1,33);u_r*ones(1,118)];
% figure(3);
% plot(x, y, '--', 'Linewidth', 1);
% xlabel('X (m)')
% ylabel('Y (m)')
% axis equal
% axis([-2 42 -2 42])
% grid on
% 
% save('ref_traj_3.mat','U_ref', 'X_ref');