% propigate state
function Xdot = learn2bal_odefcn_wheelie(t,X,u,p)  %(time, state vector, control input, and parameter struct)

% deconstruct state vector
x         = X(1);
x_dot     = X(2);
theta     = X(3);
theta_dot = X(4);

% account for speed reduction (or torque reduction) from pulley configuration
Tm1 = u;                                     % motor torque exterted on pendulum
Tm2 = (p.n_grooves_2/p.n_grooves_1)*Tm1;      % torque exerted by belt on axle pulley

% evaluate constants
x_bar = p.l_cm*cos(theta);
y_bar = p.l_cm*sin(theta);
A = p.Ip + p.mp*p.l_cm^2;
B = p.mp*y_bar;
C = Tm1 - p.mp*p.g*x_bar;
D = p.Ic + (p.mp+p.mc)*p.r_wheel^2;
E = p.mp*p.r_wheel^2*y_bar;
F = p.r_wheel*Tm2 + p.mp*p.r_wheel^2*x_bar*theta_dot^2;

% construct Xdot from differential equations
% note: X = [x x_dot theta theta_dot] therefore Xdot = [x_dot x_ddot theta_dot theta_ddot]
Xdot = zeros(4,1);
Xdot(1,:) = x_dot;
Xdot(2,:) = (C*E+A*F)/(A*D-B*E);
Xdot(3,:) = theta_dot;
Xdot(4,:) = (B*F+C*D)/(A*D-B*E);

end % end function