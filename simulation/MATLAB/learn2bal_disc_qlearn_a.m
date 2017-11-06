% restart
close all; clear all; clc;

global discX; % share discrete state history with plotting function; TODO remove this, it is a hack

% plotting options
plotOpts.doSaveFrames = 0;
plotOpts.showEveryN = 0;

% initialize parameters
sysParams = learn2bal_get_params();

% calculate critical speed for entering endo mode to passively reach vertical
theta_end = 80*pi/180;
Icombined = sysParams.Ic + sysParams.Ip + (sysParams.mc + sysParams.mp)*sysParams.r_wheel^2 + sysParams.mp*(sysParams.l_cm^2 + 2*sysParams.r_wheel*sysParams.l_cm*sin(sysParams.theta0));
B = (sysParams.Ic/sysParams.r_wheel + sysParams.mp*sysParams.l_cm*sin(sysParams.theta0) + (sysParams.mc+sysParams.mp)*sysParams.r_wheel);
omega_post = sqrt(2*sysParams.mp*9.81*sysParams.l_cm*(sin(theta_end)-sin(sysParams.theta0))/Icombined);
x_dot_crit = (Icombined*omega_post/B);       % [m/s]

% initial conditions X0 = [x0 xdot0]'
X0 = [0 0 sysParams.theta0  0]'; % [m m rad rad/s]'
X = X0;
sim_mode = l2b_mode.drive;

% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 2;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% data storage: state at time t
time        = t0;
X_data      = X0;
energy_data = learn2bal_compute_energy(X,sysParams);  % compute initial total energy in system

% state discritization
discStateValsX = [0:1:180]*pi/180;      % [rad]
discStateValsY = [-1600:50:1600]*pi/180;   % [rad/s]
discStateValsZ = [-20:0.25:20];            % [m/s]
nx = length(discStateValsX);
ny = length(discStateValsY);
nz = length(discStateValsZ);

% action discritization
uMax = 0.2;
discActionVals = [-1 -0.5 -0.25 -0.125 0 0.125 0.25 0.5 1]*uMax;

% Q table
%Q = zeros(nx*ny*nz,length(discActionVals)+1);
Q = unidrnd(2,nx*ny*nz,length(discActionVals)+1)-1;

% data storage for discrete state info
[stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
discStateN = stateNum;
discX = dStates;

% learning parameters
epsilon = 0.2;   % exploration vs. exploitation control
alpha = 0.01;    % learning rate
gamma = 0.2;     % discount factor

% data storage: control effort and mode to transition from state at time t to state at time t+1
% (last element of u_data will be null, last element of mode_data will be l2b_mode.complete)
u_data      = [];
mode_data   = {sprintf("%s",sim_mode)};

% run simulation
for t = t0:dt:(tf-dt)
    
    % extract values from current state vector
    x = X(1);
    x_dot = X(2);
    theta = X(3);
    theta_dot = X(4);
    
    [stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
    actionCosts = Q(stateNum,:)
    % take an action
    if( unifrnd(0,1) < epsilon)
        [~,aIdx] = min(actionCosts);
    else
        % choose an action at random
        aIdx = unidrnd(length(actionCosts));
    end
    
    if(aIdx == length(actionCosts))
        % compute new state vector that conserves angular momentum
        % about ground contact point during "collision" when brake applied
        A = sysParams.Ip + sysParams.mp*(sysParams.l_cm^2+sysParams.r_wheel*sysParams.l_cm*sin(theta));
        B = (sysParams.Ic/sysParams.r_wheel + sysParams.mp*sysParams.l_cm*sin(theta) + (sysParams.mc+sysParams.mp)*sysParams.r_wheel);
        C = sysParams.Ic + sysParams.Ip + (sysParams.mc + sysParams.mp)*sysParams.r_wheel^2 + sysParams.mp*(sysParams.l_cm^2 + 2*sysParams.r_wheel*sysParams.l_cm*sin(theta));
        theta_dot_1 = (A*theta_dot - B*x_dot)/(C);
        X = [x -theta_dot_1*sysParams.r_wheel  theta theta_dot_1]';
        
        % start propigating state in endo mode
        sim_mode = l2b_mode.endo;
        u = 0;
    else
        u = discActionVals(aIdx);
    end
    
    % propigate state, keeping only the final state returned by the ODE solver
    [T, X, u_applied, sim_mode] = learn2bal_run_sim_step(t,X,u,sysParams,sim_mode,[t t+dt]);
    X_prime = X(end, :)';  % note: this step is necessary to keep state vector dimensions correct for next call to ode45()
    
    % get reward
    
    % update Q function
    Q(stateNum,aIdx) = Q(stateNum,aIdx) + 
    
    X = X_prime;
    
    % store results from this timestep
    time(end+1)      = T(end);
    X_data(:,end+1)  = X;          % note: discarding state values at intermediate timesteps calculated by ode45()
    u_data(:,end+1)  = u_applied;
    mode_data{end+1} = sprintf("%s",sim_mode);  % this is the mode at the END of the simulated timestep; i.e. mode used for NEXT propigation
    energy_data(:,end+1)  = learn2bal_compute_energy(X,sysParams);
    
    % data storage for discrete state info
    [stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
    discStateN(end+1) = stateNum;
    discX(:,end+1) = dStates;
    
    % determine and store discrete state information
    
    % break out of loop if system crashes back to initial pendulum angle
    % TODO: handle this collision and return to drive mode...
    if(sim_mode == l2b_mode.crash)
        warning('Crash condition detected');
        break
    end
end

% add null control input and mode data for last state (not transitioning from last state...)
u_data(end+1)    = 0;
mode_data{end}   = sprintf("%s",l2b_mode.complete); % overwrite...

% plot results
learn2bal_plot(plotOpts, sysParams, time, X_data, u_data, mode_data, energy_data);