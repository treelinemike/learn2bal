function learn2bal_plot(plotOpts, sysParams, time, X_data, u_data, mode_data, energy_data)

%% plot time series trajectories
figure;
set(gcf,'Position',[3.218000e+02 4.660000e+01 8.248000e+02 7.376000e+02]);
ax = subplot(5,1,1);
hold on; grid on;
plot(time,X_data(1,:),'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfPosition [m]','FontSize',12);
title('\bfSimulation Results','FontSize',14);

ax(end+1) = subplot(5,1,2);
hold on; grid on;
plot(time,X_data(2,:),'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfVelocity [m/s]','FontSize',12);

ax(end+1) = subplot(5,1,3);
hold on; grid on;
plot(time,X_data(3,:)*180/pi,'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bf\Theta [deg]','FontSize',12);

ax(end+1) = subplot(5,1,4);
hold on; grid on;
plot(time,X_data(4,:)*180/pi,'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bf\Theta dot [deg/s]','FontSize',12);

ax(end+1) = subplot(5,1,5);
hold on; grid on;
plot(time,u_data,'b-','LineWidth',1.6);
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfMotor Torque [Nm]','FontSize',12);

linkaxes(ax,'x');

%% plot energy breakdown at each timestep
if( size(energy_data,2) == size(time,2) )
    figure;
    set(gcf,'Position',[1.426000e+02 3.634000e+02 1.189600e+03 3.032000e+02]);
    hold on; grid on;
    
    plot(time,sum(energy_data),'r','LineWidth',3);
    plot(time,energy_data,'LineWidth',1.3);
    legend('Total','Wheel Rotational','Wheel Linear','Body Rotational','Body Linear','Potential','Location','SouthEast');
    xlabel('\bfTime [s]');
    ylabel('\bfEnergy [J]');
    title('\bfTotal Energy in System','FontSize',12);
end

%% plot pseudo torque-speed curve
% TODO: make sure we're dealing with motor speeed and torque, not speed and
% torque at wheel
figure
hold on; grid on;
plot( (abs(X_data(2,:))/sysParams.r_wheel)*(10/(2*pi))  ,abs(u_data))
xlabel('\bfSpeed (rpm)');
ylabel('\bfTorque (Nm)');

%% animate results
figure;
set(gcf,'Position',[0029 1.378000e+02 1.446400e+03 0624]);
hold on; grid on;
frameCount = 1;

for i = [1:plotOpts.showEveryN:size(X_data,2) size(X_data,2)]
    
    % get current state
    x = X_data(1,i);
    x_dot = X_data(2,i);
    theta = X_data(3,i);
    theta_dot = X_data(4,i);
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
    if(plotOpts.doSaveFrames)
        saveas(gcf,sprintf('frame%03d.png',frameCount));
        frameCount = frameCount + 1;
    else
        pause(0.01);
    end
    
end

end