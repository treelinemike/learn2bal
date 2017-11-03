function U = learn2bal_compute_energy(X,p) % X = state vector, p = model parameter structure

% compute angular velocity of wheel
phi_dot = -X(2)/p.r_wheel;

% compute linear velocity of pendulum center of mass
v_pcm_sq = (X(2)^2 -2*X(4)*X(2)*p.l_cm*sin(X(3)) + p.l_cm^2*X(4)^2);

% compute each component of energy in the system in this state
U = [
    0.5*p.Ic*(phi_dot)^2; ...      % kinetic energy, wheel rotation
    0.5*p.mc*X(2)^2;  ...          % kinetic energy, wheel translation
    0.5*p.Ip*X(4)^2; ...           % kinetic energy, pendulum rotation
    0.5*p.mp*v_pcm_sq; ...         % kinetic energy, pendulum translation
    p.mp*9.81*p.l_cm*sin(X(3))     % potential energy, gravitational
    ];

end