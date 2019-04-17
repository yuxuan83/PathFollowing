function dxdt = vehicle_dynamics(x, u)
    % Vehicle parameters
    lf = 1.105; % center to front wheel length [m]
    lr = 1.738; % center to rear wheel length [m]
    L = lf + lr; % front wheel to rear wheel length [m]
    
    dxdt = [x(4)*cos(x(3)) - (lr/L)*x(4)*tan(u(1))*sin(x(3)); ...
            x(4)*sin(x(3)) + (lr/L)*x(4)*tan(u(1))*cos(x(3)); ...
            x(4)*tan(u(1))/L; ...
            u(2)];
end