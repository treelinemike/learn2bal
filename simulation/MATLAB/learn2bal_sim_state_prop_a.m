% propigate state
function Xdot = ekfTestStateProp(t,X,u)

% stiffness and damping parameters
global mp mc g Ip Ic l_cm l_link b r_wheel n_grooves_1 n_grooves_2;

% deconstruct state vector
s_x       = X(1);
v_x       = X(2);
theta     = X(3);
theta_dot = X(4);

% account for speed reduction (or torque reduction) from pulley configuration
Tm1 = u;                                     % motor torque exterted on pendulum
Tm2 = (n_grooves_2/n_grooves_1)*Tm1;         % torque exerted by belt on axle pulley

% evaluate constants
x_bar = l_cm*cos(theta);
y_bar = l_cm*sin(theta);
A = Ip + mp*l_cm^2;
B = mp*y_bar;
C = Tm1 - mp*g*x_bar;
D = Ic + (mp+mc)*r_wheel^2;
E = mp*r_wheel^2*y_bar;
F = r_wheel*Tm2 + mp*r_wheel^2*x_bar*theta_dot^2;

% construct Xdot from differential equation
% note: X = [theta theta_dot] therefore Xdot = [theta_dot theta_ddot]
Xdot = zeros(4,1);
Xdot(1,:) = v_x;
Xdot(2,:) = (C*E+A*F)/(A*D-B*E);
Xdot(3,:) = theta_dot;
Xdot(4,:) = (B*F+C*D)/(A*D-B*E);