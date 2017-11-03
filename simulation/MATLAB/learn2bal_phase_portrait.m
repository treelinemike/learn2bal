% restart
close all; clear all; clc;

% initialize parameters
sysParams = learn2bal_get_params();

for i = 1:2

    if(i == 1)
        sim_mode = l2b_mode.free;
    else
        sim_mode = l2b_mode.endo_free;
    end
    
dtheta = 2*pi/10;
thetas = -pi/2:3*pi/2;
for theta0 = thetas
% initial conditions X0 = [x0 xdot0]'
X0 = [0 0 theta0  0]'; % [m m rad rad/s]'
X = X0;

% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 2;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% data storage: state at time t
time        = t0;
X_data      = X0;

% run simulation
for t = t0:dt:(tf-dt)
    
    u=0;

    % propigate state, keeping only the final state returned by the ODE solver
    [T, X, u_applied, sim_mode] = learn2bal_run_sim_step(t,X,u,sysParams,sim_mode,[t t+dt]);
    X = X(end, :)';  % note: this step is necessary to keep state vector dimensions correct for next call to ode45()
    
    % store results from this timestep
    time(end+1)      = T(end);
    X_data(:,end+1)  = X;          % note: discarding state values at intermediate timesteps calculated by ode45()
    
end


if(i == 1)
    color = [0 0 1];
else
    color = [1 0 0];
end

% plot results
hold on; grid on;
plot3(X_data(3,:),X_data(4,:),X_data(2,:),'Color',color);
xlabel('Theta [rad]');
ylabel('Theta_dot [rad/s]');
zlabel('x_dot [m/s]');
hold on; grid on;
plot3(-pi/2,0,0,'g.','MarkerSize',25);
plot3(pi/2,0,0,'r.','MarkerSize',25);
drawnow;
end
end