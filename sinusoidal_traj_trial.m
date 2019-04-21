
dt = 0.05; %time step (sec)
T = 5; 
v_x = 12; %m/s
dx = v_x*dt;
x_f_s = T*v_x;
x_r_s = 0:dx:x_f_s;
y_r_s = zeros(1,length(x_r_s));


%% Generate a straight trajectory
% U_ref = zeros(2,101);
% X_ref = [x_r_s; y_r_s;zeros(1,101);v_x*ones(1,101)];
% figure(1);
% plot(x_r_s, y_r_s, '*', 'Linewidth', 1);
% savefile = 'ref_traj_1.mat';
% save(savefile, 'U_ref', 'X_ref');
%% Generate sinusoidal trajectory 
x_f = T*v_x;
x_r = 0:dx:x_f;
y_r = 3.2*sin(x_r*(2*pi/(v_x*T)));
psi_r = 3.2*(2*pi/(v_x*T))*cos(x_r*(2*pi/(v_x*T)));
u_r = v_x * ones(1,101);
U_ref = [zeros(1,101);zeros(1, 101)];
X_ref = [x_r;y_r;psi_r;u_r];
savefile = 'ref_traj_2.mat';
save(savefile, 'U_ref', 'X_ref');
figure(2);
plot(x_r, y_r, '--', 'Linewidth', 1);
ylim([-6, 6]);
grid on;
axis equal
figure(3);
plot(x_r, u_r);