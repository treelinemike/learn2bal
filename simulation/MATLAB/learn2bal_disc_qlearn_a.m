% TODO:
% - display mode traces vs. time
% - visualize greedy actions in state space with switching surfaces
% - save and recall policies

% restart
close all; clc; clear all;

global discX; % share discrete state history with plotting function; TODO remove this, it is a hack

% number of training trials
nTrainTrials = 1000;

% plotting options
plotOpts.doSaveFrames = 0;
plotOpts.showEveryN = 15;

% initialize parameters
sysParams = learn2bal_get_params();

% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 2;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% learning parameters
epsilon = 0.2;   % exploration vs. exploitation control  % may want to make higher (0.9) then reduce to a minimum
% adjacency trace ???
alpha = 0.6;      % learning rate
gamma = 0.6;      % discount factor
lambda = 0.1;     % decay factor on eligibility traces

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
% discActionVals = [-1 -0.2 0 0.2 1]*uMax;

% initialzie Q table randomly
%Q = zeros(nx*ny*nz,length(discActionVals)+1);
% Q = unidrnd(2,nx*ny*nz,length(discActionVals)+1)-1;

% Q = zeros(nx*ny*nz,length(discActionVals));
Q = unidrnd(2,nx*ny*nz,length(discActionVals))-1;
e = zeros(nx*ny*nz,length(discActionVals));

exploitExplore = [];


for i = 1:nTrainTrials
    
    % discount epsilon
    epsilon = 0.999*epsilon;
    
    % set epsilon to zero (greedy policy) for test runs
    if(i == nTrainTrials)
        epsilon = 0;
    end
    
    
    % initial conditions
    if(i == nTrainTrials)
        X0 = [0 0 80*pi/180 0]'; % [m m/s rad rad/s]';
    else
        X0 = [0; 0; (90+(randn()*10/1.96))*pi/180; ((randn()*200/1.96))*pi/180];
        X0*180/pi;
    end
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
        actionRewards = Q(stateNum,:);
        
        % determine which action to take from here
        if( unifrnd(0,1) > epsilon)
            % EXPLOIT
            exploitExplore(end+1) = 1;
            [~,aIdx] = max(actionRewards);
        else
            % EXPLORE
            % choose an action at random
            exploitExplore(end+1) = 2;
            aIdx = unidrnd(length(actionRewards));  % NOTE: NOT ALLOWING SELECTION OF BRAKE LOCK!!!
        end
        
        % compute appropriate control action
        if(0 ) %aIdx == length(actionRewards))
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
        
        % penalize being away from top
%         if( abs(pi/2 - X_current(3)) < (5*pi/180))
%             c = 5000;
%         end
        if( abs(pi/2 - X_current(3)) > (10*pi/180))
            c = -500000;
        end
        
%         % penalize locking brakes
%         if(aIdx == 4)
%             c = c - 100;
%         end
        
        % penalize wheel velocity
        if( abs(X_current(2)) > 0.1)
            c = -100;
        end
        
%         penalize changing modes and changing control action
        if( (sim_mode ~= mode_data(end)) || (~isempty(u_data) && (u ~= u_data(end))) )
            c = c - 100;
        end
        
        % update Q table
        if(i ~= nTrainTrials)
            Q(stateNum,aIdx) = (1-alpha)*Q(stateNum,aIdx) + alpha*(c + gamma*(max(Q(stateNum_prime,:))));
        end 
        
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
    %      time(end)
end

Q(discStateN(1:10),:)

% plot results
learn2bal_plot(plotOpts, sysParams, time, X_data, u_data, mode_data, []);

%% plot policy
figure
[~,policyVec] = max(Q,[],2);
idxX = repmat(discStateValsX',length(discStateValsY)*length(discStateValsZ),1)*180/pi;
idxY = repmat(  repelem(discStateValsY',length(discStateValsX),1) , length(discStateValsZ), 1)*180/pi;
idxZ = repelem(discStateValsZ',length(discStateValsX)*length(discStateValsY),1);
stateIdx = [idxX, idxY, idxZ];
scatter3(idxX,idxY,idxZ,50,policyVec,'filled')
xlabel('Body Angle [deg]');
ylabel('Body Angular velocity [deg/s]');
zlabel('Horizontal Speed [m/s]');
colorbar
hold on;

set(gca,'ZLim',[0.01 0.21])
plot3([90 90],[-220 220],[0.2 0.2],'r-','LineWidth',3);

% set(gca,'ZLim',[-0.01 0.01])
% plot3([90 90],[-220 220],[0 0],'r-','LineWidth',3);

% set(gca,'ZLim',[-0.41 -0.21])
% plot3([90 90],[-220 220],[-0.4 -0.4],'r-','LineWidth',3);


view([0 90])


% save Q
% save('Q004.mat','Q')