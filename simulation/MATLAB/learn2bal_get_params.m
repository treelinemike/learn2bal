function sysParams = learn2bal_get_params()

sysParams.l_link       = 0.19;      % [m]
sysParams.l_cm         = 0.072;     % [m] distance from axle to CM of pendulum assy
sysParams.mp           = 0.41;      % [kg] link plus motor   (5kg = 11lb)
sysParams.Ip           = 4.94e-4;   % [kg*m^2] about pendulum CM
sysParams.r_wheel      = 0.04;      % [m]
sysParams.mc           = 0.2202;    % [kg]
sysParams.Ic           = 1.35e-4;   % [kg*m^2] about wheel assy CM
sysParams.n_grooves_1  = 20;        % number of grooves on pulley attached to motor
sysParams.n_grooves_2  = 20;        % number of grooves on pulley attached to wheels
sysParams.g            = 9.81;      % [m/s^2] 9.81m/s^2 on surface of earth
sysParams.theta0       = 0*pi/180;  % [rad] angle between body and ground in drive mode

end