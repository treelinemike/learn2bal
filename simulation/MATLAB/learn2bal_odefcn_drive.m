% propigate state
function Xdot = learn2bal_odefcn_drive(t,X,u,p)  %(time, state vector, control input, and parameter struct)

% pass out x_ddot via a global variable (not part of state vector)
global g_x_ddot;

% deconstruct state vector
x         = X(1);
x_dot     = X(2);
theta     = X(3);
theta_dot = X(4);

% account for speed reduction (or torque reduction) from pulley configuration
Tm1 = u;                                     % motor torque exterted on pendulum
Tm2 = (p.n_grooves_2/p.n_grooves_1)*Tm1;      % torque exerted by belt on axle pulley

% compute acceleration
g_x_ddot = p.r_wheel*Tm2 / ( (p.mc + p.mp)*p.r_wheel^2 + p.Ic);

% construct Xdot from differential equations
% note: X = [x x_dot theta theta_dot] therefore Xdot = [x_dot x_ddot theta_dot theta_ddot]
Xdot = zeros(4,1);
Xdot(1,:) = x_dot;
Xdot(2,:) = g_x_ddot;
Xdot(3,:) = 0;
Xdot(4,:) = 0;

end % end function