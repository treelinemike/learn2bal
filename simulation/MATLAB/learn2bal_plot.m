function learn2bal_plot(plotOpts, sysParams, time, X_data, u_data, mode_data, energy_data)

global discX; % TODO: remove

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
if(~isempty(discX))
    plot(time,discX(3,:),'r-','LineWidth',1.6);
end
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bfVelocity [m/s]','FontSize',12);

ax(end+1) = subplot(5,1,3);
hold on; grid on;
plot(time,X_data(3,:)*180/pi,'b-','LineWidth',1.6);
if(~isempty(discX))
    plot(time,discX(1,:)*180/pi,'r-','LineWidth',1.6);
end
xlabel('\bfTime [s]','FontSize',12);
ylabel('\bf\Theta [deg]','FontSize',12);


ax(end+1) = subplot(5,1,4);
hold on; grid on;
plot(time,X_data(4,:)*180/pi,'b-','LineWidth',1.6);
if(~isempty(discX))
    plot(time,discX(2,:)*180/pi,'r-','LineWidth',1.6);
end
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

%% overlay trajectory on phase portrait
%%
load('orbits.mat');
figure;
hold on; grid on;
xlabel('\bfTheta [rad]');
ylabel('\bfTheta dot [rad/s]');
zlabel('\bfx dot [m/s]');

plot3(-pi/2,0,0,'k.','MarkerSize',40);
plot3(pi/2,0,0,'ko','MarkerSize',10,'LineWidth',4);

legPlots = [];

for i = 1:2
    if(i == 1)
        data = orbits.wheelie;
        plot_color = 'b';
    else
        data = orbits.endo;
        plot_color = 'r';
    end
    
    for j = 1:size(data,2)
        this_X = data{j};
        ph = plot3(this_X(3,:),this_X(4,:),this_X(2,:),'Color',plot_color);
        if(j == 1)
            legPlots(end+1) = ph;
        end
    end
    
end

legPlots(end+1) = plot3(X_data(3,:),X_data(4,:),X_data(2,:),'-','LineWidth',3','Color',[ 0 0.7 0]);
plot3(X_data(3,1),X_data(4,1),X_data(2,1),'o','MarkerSize',8,'MarkerFaceColor',[0 1 0],'MarkerEdgeColor',[0 0.7 0],'LineWidth',2);
plot3(X_data(3,end),X_data(4,end),X_data(2,end),'o','MarkerSize',8,'MarkerFaceColor',[1 0 0],'MarkerEdgeColor',[0 0.7 0],'LineWidth',2);

legend(legPlots,{'Wheelie Orbits','Endo Orbits','Actual Trajectory'});
xlim([-0.05 1.8]);
ylim([-20 20]);

%% animate results
figure;
set(gcf,'Position',[0029 1.378000e+02 1.446400e+03 0624]);
% set(gcf,'Position',[0029 4.546000e+02 8.144000e+02 3.072000e+02]);
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
    plot([-10 10],[0 0],'-','Color',[0 0.7 0],'LineWidth',2.0);
    hold on; grid on;
    
    if(plotOpts.doSaveFrames && frameCount == 1)
        grid on;
        set(gca,'YLim',[-0.025 0.25]);
        set(gca,'XLim',[-.5 0.2]);
        saveas(gcf,sprintf('frame0.png',frameCount));
    end
    
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
    
    % adjut axis limits
    set(gca,'YLim',[-0.25 0.3]);
    set(gca,'XLim',[-.6 0.6]);
%     set(gca,'YLim',[-0.025 0.25]);
%     set(gca,'XLim',[-.5 0.2]);
    
    
    % draw plot and delay
    drawnow;
    if(plotOpts.doSaveFrames)
        grid off;
        saveas(gcf,sprintf('frame%03d.png',frameCount));
        frameCount = frameCount + 1;
    else
        pause(0.01);
    end
    
end

end