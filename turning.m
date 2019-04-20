%% 
R = 20;
u_r = 12;
T = 5;
dt = 0.05;
theta = 0:1.2/40:pi/2 -1.2/40;
x = [zeros(1,33),R - R*cos(theta), R:R/33:2*R - R/33];
y = [0:R/33:R-R/33, R + R*sin(theta), 2*R*ones(1,33)];
U_ref = [zeros(1,118);zeros(1,118)];
X_ref = [x;y;pi/2*ones(1,33), pi/2 - theta, zeros(1,33);u_r*ones(1,118)];
figure(1);
plot(x, y, '--', 'Linewidth', 1);
axis equal;
save('ref_traj_3.mat','U_ref', 'X_ref');