% restart
close all; clear all; clc;

% options
doSaveFrames = 0;

% initialize parameters
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
sysParams.theta0       = 0*pi/180; % [rad] angle between body and ground in drive mode

% calculate critical speed for entering endo mode to passively reach vertical
theta_end = 85*pi/180;
Icombined = sysParams.Ic + sysParams.Ip + (sysParams.mc + sysParams.mp)*sysParams.r_wheel^2 + sysParams.mp*(sysParams.l_cm^2 + 2*sysParams.r_wheel*sysParams.l_cm*sin(sysParams.theta0));
B = (sysParams.Ic/sysParams.r_wheel + sysParams.mp*sysParams.l_cm*sin(sysParams.theta0) + (sysParams.mc+sysParams.mp)*sysParams.r_wheel);
omega_post = sqrt(2*sysParams.mp*9.81*sysParams.l_cm*(sin(theta_end)-sin(sysParams.theta0))/Icombined);
sysParams.xdotCrit = (Icombined*omega_post/B);       % [m/s]

% initial conditions X0 = [x0 xdot0]'
X0 = [0 0 sysParams.theta0  0]'; % [m m rad rad/s]'
X = X0;
sim_mode = l2b_mode.drive;

% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 2;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% data storage
time = [t0];
data = [X0];
uData = [];  % u to appled between CURRENT timestep and NEXT timestep (last element will be zero)
modeData = {};

% compute initial total energy
phi_dot = -X(2)/sysParams.r_wheel;
v_pcm_sq = (X(2)^2 -2*X(4)*X(2)*sysParams.l_cm*sin(X(3)) + sysParams.l_cm^2*X(4)^2);
Utotal = [0.5*sysParams.Ic*(phi_dot)^2; 0.5*sysParams.mc*X(2)^2; 0.5*sysParams.Ip*X(4)^2; 0.5*sysParams.mp*v_pcm_sq; sysParams.mp*9.81*sysParams.l_cm*sin(X(3))];

% run simulation
for t = t0:dt:(tf-dt)
    
    % generate control input (if necessary)
    switch(sim_mode)
        case l2b_mode.drive
            u = -0.1;
            
            % switch to endo mode if we're going fast enough
            if( (sign(X(2)) ~= sign(pi/4-X(3))) & (abs(X(2)) > sysParams.xdotCrit*1.0) )
                
                % compute new state vector that conserves angular momentum
                % about ground contact point during "collision" when brake
                % is applied
                x = X(1);
                x_dot = X(2);
                theta = X(3);
                theta_dot = X(4);
                
                A = sysParams.Ip + sysParams.mp*(sysParams.l_cm^2+sysParams.r_wheel*sysParams.l_cm*sin(theta));
                B = (sysParams.Ic/sysParams.r_wheel + sysParams.mp*sysParams.l_cm*sin(theta) + (sysParams.mc+sysParams.mp)*sysParams.r_wheel);
                C = sysParams.Ic + sysParams.Ip + (sysParams.mc + sysParams.mp)*sysParams.r_wheel^2 + sysParams.mp*(sysParams.l_cm^2 + 2*sysParams.r_wheel*sysParams.l_cm*sin(theta));
                theta_dot_1 = (A*theta_dot - B*x_dot)/(C);
                X = [x -theta_dot_1*sysParams.r_wheel  theta theta_dot_1]';
                
                % start propigating state in endo mode
                sim_mode = l2b_mode.endo;
                
            end
            
        case l2b_mode.endo
            % no control input in endo mode
            u = 0;
            
            % switch to wheelie mode when
            if( abs(X(3) - pi/2) < 5*pi/180 )
                sim_mode = l2b_mode.wheelie;
            end
            
        case l2b_mode.wheelie
            
            % generate control input
            Kd_x      =  2.7;        %1.5 7..... +0.8 ... negative??
            Kp_theta  =  1.8;   %5.0 10 .... +5.0 ... positive
            Kd_theta  =  -1;   %-0.5 -2 .... -0.5 ... negative
            u         =  Kd_x*(X(2)) + Kp_theta*((pi/2)-X(3)) + Kd_theta*(X(4));  %+ Ki_x*(x_int)
            
        case l2b_mode.free
            u = 0;
            
        otherwise
            error('Cannot simulate from mode: %s', sim_mode);
    end
    
    % apply torque limits
    % TODO: make torque limits velocity dependent
    if ( abs(u) > 0.2 )
        u = sign(u)*0.2;
    end
    
    % calculate timestep for ODE solving
    odeTime = [t t+dt];
    
    % propigate state
    [T, X, sim_mode] = learn2bal_run_sim_step(t,X,u,sysParams,sim_mode,odeTime);
    X = X(end, :)';  % note: this step is necessary to keep state vector dimensions correct for next call to ode45()
    
    % store results from this timestep
    time(end+1)   = T(end);
    data(:,end+1) = X; % note: discarding state values at intermediate timesteps calculated by ode45()
    uData(:,end+1) = u;
    
    modeData{end+1} = sim_mode;
    
    % compute and save total energy
    phi_dot = -X(2)/sysParams.r_wheel;
    v_pcm_sq = (X(2)^2 - 2*X(4)*X(2)*sysParams.l_cm*sin(X(3)) + sysParams.l_cm^2*X(4)^2);
    Utotal(:,end+1) = [0.5*sysParams.Ic*(phi_dot)^2; 0.5*sysParams.mc*X(2)^2; 0.5*sysParams.Ip*X(4)^2; 0.5*sysParams.mp*v_pcm_sq; sysParams.mp*9.81*sysParams.l_cm*sin(X(3))];
    
    
    % break out of loop if crash
    if(sim_mode == l2b_mode.crash)
        warning('Crash condition detected');
        break
    end
    
end
uData(end+1) = 0; % u to appled between CURRENT timestep and NEXT timestep (last element will be zero)


%% plot results
figure;
set(gcf,'Position',[3.218000e+02 4.660000e+01 8.248000e+02 7.376000e+02]);
ax = subplot(5,1,1);
hold on; grid on;
plot(time,data(1,:),'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfPosition [m]','FontSize',12);
title('\bfSimulation Results','FontSize',14);

ax(end+1) = subplot(5,1,2);
hold on; grid on;
plot(time,data(2,:),'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfVelocity [m/s]','FontSize',12);

ax(end+1) = subplot(5,1,3);
hold on; grid on;
plot(time,data(3,:)*180/pi,'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bf\Theta [deg]','FontSize',12);

ax(end+1) = subplot(5,1,4);
hold on; grid on;
plot(time,data(4,:)*180/pi,'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bf\Theta dot [deg/s]','FontSize',12);

ax(end+1) = subplot(5,1,5);
hold on; grid on;
plot(time,uData,'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfMotor Torque [Nm]','FontSize',12);

linkaxes(ax,'x');

%% show energy plot
figure;
set(gcf,'Position',[1.426000e+02 3.634000e+02 1.189600e+03 3.032000e+02]);
hold on; grid on;

plot(time,sum(Utotal),'r','LineWidth',3);
plot(time,Utotal,'LineWidth',1.3);
legend('Total','Wheel Rotational','Wheel Linear','Body Rotational','Body Linear','Potential','Location','SouthEast');
xlabel('\bfTime [s]');
ylabel('\bfEnergy [J]');
title('\bfTotal Energy in System','FontSize',12);
%%
figure
hold on; grid on;
plot( (abs(data(2,:))/sysParams.r_wheel)*(10/(2*pi))  ,abs(uData))
xlabel('\bfSpeed (rpm)');
ylabel('\bfTorque (Nm)');

%% visaulization
figure;
set(gcf,'Position',[0029 1.378000e+02 1.446400e+03 0624]);
hold on; grid on;
frameCount = 1;

for i = [1:20:size(data,2) size(data,2)]
    
    % get current state
    x = data(1,i);
    x_dot = data(2,i);
    theta = data(3,i);
    theta_dot = data(4,i);
    
    phi = -x/sysParams.r_wheel;   % [rad]
    
    % draw ground
    hold off;
    plot([-10 10],[0 0],'-','Color',[0 0.7 0]);
    hold on; grid on;
    
    % draw wheel
    for j = 1:4
        gamma = ((j-1):0.01:j)*pi/2;
        x_wheel = x+sysParams.r_wheel*cos(gamma + phi);
        y_wheel = sysParams.r_wheel + sysParams.r_wheel*sin(gamma + phi);
        
        linecolor = 'r';
        if(mod(j,2))
            linecolor = 'k';
        end
        
        plot(x_wheel,y_wheel,'-','Color',linecolor,'LineWidth',5);
    end
    
    % draw pendulum
    plot(x+[0 sysParams.l_link*cos(theta)],sysParams.r_wheel+[0 sysParams.l_link*sin(theta)],'b-','LineWidth',5);
    
    % draw center of wheel
    plot(x,sysParams.r_wheel,'k.','MarkerSize',25);
    
    % adjut axis limits and delay
    set(gca,'YLim',[-0.25 0.3]);
    set(gca,'XLim',[-.6 0.6]);
    drawnow;
    if(doSaveFrames)
        saveas(gcf,sprintf('frame%03d.png',frameCount));
        frameCount = frameCount + 1;
    else
        pause(0.01);
    end
    
end
