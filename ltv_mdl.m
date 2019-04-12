function [A, B] = ltv_mdl(x_ref, u_ref)
    % Vehicle parameters
    lf = 1.105; % center to front wheel length [m]
    lr = 1.738; % center to rear wheel length [m]
    L = lf + lr; % front wheel to rear wheel length [m]
    b = lr / L;
    
    A11 = 0;
    A12 = 0;
    A13 = -x_ref(4) * sin(x_ref(3)) - b * x_ref(4) * cos(x_ref(3)) * tan(u_ref(1));
    A14 = cos(x_ref(3)) - b * sin(x_ref(3)) * tan(u_ref(1));
    A21 = 0;
    A22 = 0;
    A23 = x_ref(4) * cos(x_ref(3)) - b * x_ref(4) * sin(x_ref(3)) * tan(u_ref(1));
    A24 = sin(x_ref(3)) + b * cos(x_ref(3)) * tan(u_ref(1));
    A31 = 0;
    A32 = 0; 
    A33 = 0;
    A34 = tan(u_ref(1))/L;
    A41 = 0; 
    A42 = 0;
    A43 = 0;
    A44 = 0;
    
    A = [A11, A12, A13, A14; ...
         A21, A22, A23, A24; ...
         A31, A32, A33, A34; ...
         A41, A42, A43, A44];
     
    B11 = -b * x_ref(4) * (sec(u_ref(1))^2) * sin(x_ref(3));
    B12 = 0;
    B21 = b * x_ref(4) * cos(x_ref(3)) * (sec(u_ref(1))^2);
    B22 = 0;
    B31 = x_ref(4) * (sec(u_ref(1))^2) / L;
    B32 = 0;
    B41 = 0;
    B42 = 1;
    
    B = [B11, B12; ...
         B21, B22; ...
         B31, B32; ...
         B41, B42];
    
end