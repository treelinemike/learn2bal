% replay from a learned Q table

% restart
close all; clc; clear all;

% load Q table
load('Q004.mat');

global discX; % share discrete state history with plotting function; TODO remove this, it is a hack

% plotting options
plotOpts.doSaveFrames = 0;
plotOpts.showEveryN = 15;

% initialize parameters
sysParams = learn2bal_get_params();

% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 2;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% state discritization
discStateValsX = [30:5:150]*pi/180;      % [rad]
discStateValsY = [-800:100:800]*pi/180;   % [rad/s]
discStateValsZ = [-1:0.2:1];             % [m/s]
nx = length(discStateValsX);
ny = length(discStateValsY);
nz = length(discStateValsZ);

% action discritization
uMax = 0.1;
discActionVals = [-1 0 1]*uMax;

% initial conditions
X0 = [0 0 80*pi/180 0]'; % [m m/s rad rad/s]';
X = X0;
sim_mode = l2b_mode.wheelie;

% data storage: state at time t
time        = t0;
X_data      = X0;

% data storage for discrete state info
[stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
discStateN = stateNum;
discX = dStates;

% data storage: control effort and mode to transition from state at time t to state at time t+1
% (last element of u_data will be null, last element of mode_data will be l2b_mode.complete)
u_data      = [];
mode_data   = sim_mode;

% run simulation
for t = t0:dt:(tf-dt)
       
    % determine index of current state
    [stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
        
    % compute appropriate control action
    [~,aIdx] = max(Q(stateNum,:));
     u = discActionVals(aIdx);
    
    % propigate state, keeping only the final state returned by the ODE solver
    [T, X, u_applied, sim_mode] = learn2bal_run_sim_step(t,X,u,sysParams,sim_mode,[t t+dt]);
    X_prime = X(end, :)';  % note: this step is necessary to keep state vector dimensions correct for next call to ode45()
    
    % get index of new state
    [stateNum_prime,idxVec_prime,dStates_prime] = learn2bal_get_disc_state(X_prime,discStateValsX,discStateValsY,discStateValsZ);
    
    % update state
    X = X_prime;
    dStates = dStates_prime;
    stateNum = stateNum_prime;
    
    % store results from this timestep
    time(end+1)      = T(end);
    X_data(:,end+1)  = X;          % note: discarding state values at intermediate timesteps calculated by ode45()
    u_data(:,end+1)  = u_applied;
    mode_data(end+1) = sim_mode;  % this is the mode at the END of the simulated timestep; i.e. mode used for NEXT propigation
    
    % data storage for discrete state info
    discStateN(end+1) = stateNum;
    discX(:,end+1) = dStates;
    
    % break out of loop if system crashes back to initial pendulum angle
    % TODO: handle this collision and return to drive mode...
    if(sim_mode == l2b_mode.crash)
        %warning('Crash condition detected');
        break
    end
end

% add null control input and mode data for last state (not transitioning from last state...)
u_data(end+1)    = 0;
mode_data(end)   = l2b_mode.complete; % overwrite...

% plot results
learn2bal_plot(plotOpts, sysParams, time, X_data, u_data, mode_data, []);
