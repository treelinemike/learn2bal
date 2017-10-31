function [T, X, sim_mode] = learn2bal_run_sim_step(t,X,u,p,sim_mode,odeTime)  %(current time, state vector, control input, parameter struct, simulation mode (drive, endo, wheelie), time bounds for ODE solver)

% retrieve acceleration via global variable
global g_x_ddot

switch (sim_mode)
    case l2b_mode.drive
        % simulate next step
        [T,X] = ode45(@(t,X) learn2bal_odefcn_drive(t,X,u,p),odeTime,X);
        
        % switch to 'wheelie' mode if acceleration exceeds threshold
        % note: acceleration retrieved via global variable b/c not part of
        % state - TODO: make this more elegant
        liftOffAcc = (p.mp * p.g * p.l_cm * cos(p.theta0) - u) / (p.mp * p.l_cm * sin(p.theta0));
        if( abs( g_x_ddot ) > abs(liftOffAcc) )
            sim_mode = l2b_mode.wheelie;
        end
            
    case l2b_mode.endo
        % simulate next step
        [T,X] = ode45(@(t,X) learn2bal_odefcn_endo(t,X,p),odeTime,X);

        % switch to 'crash' mode if we hit our maximum angle on either side
        theta = X(end,3);      
        if( abs((pi/2) - theta) > abs((pi/2) - p.theta0) )
            sim_mode = l2b_mode.crash;  % eventually model this collision and continue...
        end
        
    case l2b_mode.wheelie
        % simulate next step
        [T,X] = ode45(@(t,X) learn2bal_odefcn_wheelie(t,X,u,p),odeTime,X);
        
        % switch to 'crash' mode if we hit our maximum angle on either side
        theta = X(end,3);      
        if( abs((pi/2) - theta) > abs((pi/2) - p.theta0) )
            sim_mode = l2b_mode.crash;  % eventually model this collision and continue...
        end
        
    otherwise
        error('Cannot simulate from mode: %s', sim_mode);
        
end % switch

end % function