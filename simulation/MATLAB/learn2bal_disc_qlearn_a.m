% restart
close all; clear all; clc;

global discX; % share discrete state history with plotting function; TODO remove this, it is a hack

% plotting options
plotOpts.doSaveFrames = 0;
plotOpts.showEveryN = 1;

% initialize parameters
sysParams = learn2bal_get_params();

% calculate critical speed for entering endo mode to passively reach vertical
theta_end = 80*pi/180;
Icombined = sysParams.Ic + sysParams.Ip + (sysParams.mc + sysParams.mp)*sysParams.r_wheel^2 + sysParams.mp*(sysParams.l_cm^2 + 2*sysParams.r_wheel*sysParams.l_cm*sin(sysParams.theta0));
B = (sysParams.Ic/sysParams.r_wheel + sysParams.mp*sysParams.l_cm*sin(sysParams.theta0) + (sysParams.mc+sysParams.mp)*sysParams.r_wheel);
omega_post = sqrt(2*sysParams.mp*9.81*sysParams.l_cm*(sin(theta_end)-sin(sysParams.theta0))/Icombined);
x_dot_crit = (Icombined*omega_post/B);       % [m/s]


% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 2;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% learning parameters
epsilon = 0.2;   % exploration vs. exploitation control  % may want to make higher (0.9) then reduce to a minimum 
% adjacency trace (
alpha = 0.1;      % learning rate
gamma = 0.2;      % discount factor

% state discritization
discStateValsX = [0:9:180]*pi/180;         % [rad]
discStateValsY = [-200:20:200]*pi/180;   % [rad/s]
discStateValsZ = [-4:0.5:4];             % [m/s]
nx = length(discStateValsX);
ny = length(discStateValsY);
nz = length(discStateValsZ);

% action discritization
uMax = 0.2;
discActionVals = [-1 0 1]*uMax;

% initialzie Q table randomly
%Q = zeros(nx*ny*nz,length(discActionVals)+1);
Q = unidrnd(2,nx*ny*nz,length(discActionVals)+1)-1;

exploitExplore = [];


for i = 1:200
    
    % discount epsilon
    epsilon = 0.99*epsilon;
    
    % initial conditions X0 = [x0 xdot0]'
    X0 = [0 0 80*pi/180 0]'; % [m m rad rad/s]'
    X = X0;
    sim_mode = l2b_mode.wheelie;
    
    % data storage: state at time t
    time        = t0;
    X_data      = X0;
    energy_data = learn2bal_compute_energy(X,sysParams);  % compute initial total energy in system
    
    % data storage for discrete state info
    [stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
    discStateN = stateNum;
    discX = dStates;
    
    % data storage: control effort and mode to transition from state at time t to state at time t+1
    % (last element of u_data will be null, last element of mode_data will be l2b_mode.complete)
    u_data      = [];
    mode_data   = {sprintf("%s",sim_mode)};
    
    % run simulation
    for t = t0:dt:(tf-dt)
        
        % extract values from current state vector
        X_current = X;
        x = X(1);
        x_dot = X(2);
        theta = X(3);
        theta_dot = X(4);
        
        % determine index of current state
        [stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,discStateValsX,discStateValsY,discStateValsZ);
        
        % get cost of each actions from current state
        % as given by the current Q table
        actionCosts = Q(stateNum,:);
        
        % determine which action to take from here
        if( unifrnd(0,1) > epsilon)
            % EXPLOIT
            exploitExplore(end+1) = 1;
            [~,aIdx] = min(actionCosts);
        else
            % EXPLORE
            % choose an action at random
            exploitExplore(end+1) = 2;
            aIdx = unidrnd(length(actionCosts));
        end
        
        % compute appropriate control action
        if(aIdx == length(actionCosts))
            % LOCK WHEEL TO BODY
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
            if(sim_mode == l2b_mode.endo)
                sim_mode = l2b_mode.wheelie;
            end
            u = discActionVals(aIdx);
        end
        
        % propigate state, keeping only the final state returned by the ODE solver
        [T, X, u_applied, sim_mode] = learn2bal_run_sim_step(t,X,u,sysParams,sim_mode,[t t+dt]);
        X_prime = X(end, :)';  % note: this step is necessary to keep state vector dimensions correct for next call to ode45()
        
        % get index of new state
        [stateNum_prime,idxVec_prime,dStates_prime] = learn2bal_get_disc_state(X_prime,discStateValsX,discStateValsY,discStateValsZ);
        
%         x_eff = X_current(2:end,:) - [0 pi/2 0]';
%         c = x_eff' * [100 0 0; 0 1000 0; 0 0 100] * x_eff + u'*[0]*u;
        c= 0;
        % penalize being in crashed state
        if( abs(pi/2 - X_current(3)) > 10*pi/180)
            c = 100;
        end
        
        % penalize changing action
        if( (sim_mode ~= mode_data(end)) || (~isempty(u_data) && (u ~= u_data(end))) )
            c = c + 10;
        end
        
        % update Q table
        Q(stateNum,aIdx) = (1-alpha)*Q(stateNum,aIdx) + alpha*(c + gamma*(min(Q(stateNum_prime,:))));
        
        % update state 
        X = X_prime;
        dStates = dStates_prime;
        stateNum = stateNum_prime;
        
        % store results from this timestep
        time(end+1)      = T(end);
        X_data(:,end+1)  = X;          % note: discarding state values at intermediate timesteps calculated by ode45()
        u_data(:,end+1)  = u_applied;
        mode_data{end+1} = sprintf("%s",sim_mode);  % this is the mode at the END of the simulated timestep; i.e. mode used for NEXT propigation
        energy_data(:,end+1)  = learn2bal_compute_energy(X,sysParams);
        
        % data storage for discrete state info
        discStateN(end+1) = stateNum;
        discX(:,end+1) = dStates;
        
        % determine and store discrete state information
        
        % break out of loop if system crashes back to initial pendulum angle
        % TODO: handle this collision and return to drive mode...
        if(sim_mode == l2b_mode.crash)
            %warning('Crash condition detected');
            break
        end
    end
    
    % add null control input and mode data for last state (not transitioning from last state...)
    u_data(end+1)    = 0;
    mode_data{end}   = sprintf("%s",l2b_mode.complete); % overwrite...
%      time(end) 
end
% plot results
learn2bal_plot(plotOpts, sysParams, time, X_data, u_data, mode_data, []);