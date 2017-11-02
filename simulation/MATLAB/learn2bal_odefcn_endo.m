% propigate state
function Xdot = learn2bal_odefcn_endo(t,X,p)  %(time, state vector, and parameter struct)

% deconstruct state vector
x         = X(1);
x_dot     = X(2);
theta     = X(3);
theta_dot = X(4);

% evaluate constants
x_bar = p.l_cm*cos(theta);
y_bar = p.l_cm*sin(theta);

phi_ddot = (-1/( (p.mc + p.mp)*p.r_wheel^2 + p.mp*p.l_cm^2 + 2*p.mp*p.r_wheel*y_bar + p.Ic + p.Ip)) * ( (p.mp*p.r_wheel*x_bar)*theta_dot^2 + p.mp*p.g*x_bar );

% construct Xdot from differential equations
% note: X = [x x_dot theta theta_dot] therefore Xdot = [x_dot x_ddot theta_dot theta_ddot]
Xdot = zeros(4,1);
Xdot(1,:) = -theta_dot*p.r_wheel;
Xdot(2,:) = -p.r_wheel*phi_ddot;
Xdot(3,:) = theta_dot;
Xdot(4,:) = -phi_ddot;

end % end function