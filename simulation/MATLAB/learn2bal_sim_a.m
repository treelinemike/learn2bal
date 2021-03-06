% simulate simple pendulum with damping using ODE45 to propigate state
% forward in time
%
% solves x_ddot = -(b/I)*theta_dot -(m*g*l_link/(2*I))*sin(theta);

% restart
close all; clear all; clc;

% global variables to be shared with state propigation function
global mp mc g Ip Ic l_cm l_link b r_wheel n_grooves_1 n_grooves_2;

% simulation time parameters
t0 = 0;                  % [s] simulation start time
tf = 5;                  % [s] simulation end time
dt = 0.001;              % [s] timestep size

% link properties
l_link = 0.19;           % [m]
l_cm = 0.0072;           % [m] distance from axle to CM of pendulum assy
mp = 0.41;               % [kg] link plus motor   (5kg = 11lb)
Ip = 4.94e-4;            % [kg*m^2] about pendulum CM

r_wheel = 0.04;          % [m]
Ic = 1.35e-4;            % [kg*m^2] about wheel assy CM
mc = 0.2202;             % [kg]
n_grooves_1 = 20;        % number of grooves on pulley attached to motor
n_grooves_2 = 20;        % number of grooves on pulley attached to wheels
% other system parameters
g = 9.81;                % [m/s^2] 9.81m/s^2 on surface of earth
b = 0.08;                % [kg*m^2/s] damping constant

% initial conditions X0 = [x0 xdot0]'
X0 = [0 0 25*pi/180 0]'; % [m m rad rad/s]'
X = X0;

% data storage
time = [t0];
data = [X0];
uData = [];  % u to appled between CURRENT timestep and NEXT timestep (last element will be zero)

% run simulation
for t = t0:dt:(tf-dt)
    
    % generate control input
    Kd_x = 7;        %1.5 7..... +0.8 ... negative??
    Kp_theta = 5;   %5.0 10 .... +5.0 ... positive
    Kd_theta = -0.5;   %-0.5 -2 .... -0.5 ... negative
    

%     Ki_x = 0;%0.5;   % negative
%     if(size(data,2) > 10)
%         x_int = sum(data(1,end-10));
%     else
%         x_int = 0;
%     end
    
    u = Kd_x*(X(2)) + Kp_theta*((pi/2)-X(3)) + Kd_theta*(X(4));  %+ Ki_x*(x_int) 
%     u = 0;
    
    % torque limits
    if ( abs(u) > 0.5 )
        u = sign(u)*0.5;
    end


    % calculate timestep for ODE solving
    odeTime = [t t+dt];
    
    % propigate state
    [T,X] = ode45(@(t,X) learn2bal_sim_state_prop_a(t,X,u),odeTime,X);
    X = X(end, :)';  % note: this step is necessary to keep state vector dimensions correct for next call to ode45()
    
    % store results from this timestep
    time(end+1)   = T(end);
    data(:,end+1) = X; % note: discarding state values at intermediate timesteps calculated by ode45()
    uData(:,end+1) = u;
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

%%
figure
hold on; grid on;
plot( (abs(data(2,:))/r_wheel)*(10/(2*pi))  ,abs(uData))
xlabel('\bfSpeed (rpm)');
ylabel('\bfTorque (Nm)');

%% visaulization
figure;
set(gcf,'Position',[0029 1.378000e+02 1.446400e+03 0624]);
hold on; grid on;


for i = 1:50:size(data,2)
    
    % get current state
    x = data(1,i);
    x_dot = data(2,i);
    theta = data(3,i);
    theta_dot = data(4,i);
  
    phi = -x/r_wheel;   % [rad]
    
    % draw ground
    hold off;
    plot([-10 10],[0 0],'-','Color',[0 0.7 0]);
    hold on; grid on;
 
    % draw wheel
    for j = 1:4
        gamma = ((j-1):0.01:j)*pi/2;
        x_wheel = x+r_wheel*cos(gamma + phi);
        y_wheel = r_wheel + r_wheel*sin(gamma + phi);
        
        linecolor = 'r';
        if(mod(j,2))
           linecolor = 'k'; 
        end
        
        plot(x_wheel,y_wheel,'-','Color',linecolor,'LineWidth',5);
    end
    
    % draw pendulum
    plot(x+[0 l_link*cos(theta)],r_wheel+[0 l_link*sin(theta)],'b-','LineWidth',5);
    
    % draw center of wheel
    plot(x,r_wheel,'k.','MarkerSize',25);
    
    % adjut axis limits and delay
    set(gca,'YLim',[-0.25 0.3]);
    set(gca,'XLim',[-.2 1.0]);
    drawnow;
    pause(0.01);
end
