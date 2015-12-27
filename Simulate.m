% Initialize boat, environment, robot, and state objects
boat = Config_Boat ;
env = Config_Env ;
robot = Config_Robot(sim) ;
command = [0,0,0,0] ; % initialize global robot command value
state = State(sim) ;

% Initialize iteration and timing management variables
i = 1 ;
nfinity = sim.T/sim.dt + 1 ;
syncFlag = false ;
frameTimer = 1 ;
sim.quit = false ;

% Prepare history for timeseries sim, or graphics for realtime sim
if(sim.type == 1)
    fprintf('\n')
    history = History(sim) ;
elseif(sim.type == 2) % initialize 2D graphics
    Init2D ;
elseif(sim.type == 3) % initialize 3D graphics
    Init3D ;
end

% Begin simulation
fprintf('Simuu Startaruu !!\n') ;
while(i <= nfinity && sim.quit == false)
    tic ;
    
    % Record or animate
    if(sim.type == 1)
        history.record(state,robot,i) ;
        i = i+1 ;
    else
        % Print values to monitor
        %fprintf('\npx: %g  |  py: %g  |  yaw: %g   |||   vx: %g  |  vy: %g  |  yawRate: %g   \n',state.p(1),state.p(2),state.th(1),state.v(1),state.v(2),state.w(1)) ;
        % Draw for 2D or 3D
        if(sim.type == 2)
            Draw2D ;
        elseif(sim.type == 3)
            if(frameTimer == sim.showFrame)
                % Delete last graphics
                delete(boatGraphic) ;
                try
                    delete(waterGraphic) ;
                end
                delete(waypointGraphic_p) ;
                delete(waypointGraphic_y) ;
                delete(waypointGraphic_y_ep) ;
                if(strcmp(boat.type, 'direct'))
                    delete(forceGraphic) ;
                    delete(forceGraphic_ep) ;
                    delete(torqueGraphic) ;
                elseif(strcmp(boat.type, 'fixed') || strcmp(boat.type, 'azi'))
                    delete(thruster_bl) ;
                    delete(thruster_bl_ep) ;
                    delete(thruster_br) ;
                    delete(thruster_br_ep) ;
                    if(strcmp(boat.type, 'fixed'))
                        delete(thruster_fl) ;
                        delete(thruster_fl_ep) ;
                        delete(thruster_fr) ;
                        delete(thruster_fr_ep) ;
                    end
                end
                Draw3D ;
                frameTimer = 1 ;
            else
                frameTimer = frameTimer+1 ;
            end
        end
    end
    
    % Compute environmental forces
    [Fg,Mg] = Gravity(env,boat,state) ;
    [Fb,Mb] = Buoyancy(env,boat,state) ;
    [Fd,Md] = Drag(env,boat,state) ;
    [Fw,Mw] = Wind(env,boat,state) ;
    [Fv,Mv] = Waves(env,boat,state) ;
    
    % Compute robot's decision, either 4 thrusts or 2 thrusts and 2 angles
    command = robot.Decide(state,boat) ;
    
    % Compute actuator dynamics based on robot decision
    if(strcmp(boat.type, 'azi'))
        [Ft,Mt] = boat.AziThrust(state, sim, command) ;
    elseif(strcmp(boat.type, 'fixed'))
        [Ft,Mt] = boat.FixedThrust(state, command) ;
    elseif(strcmp(boat.type, 'direct'))
        [Ft,Mt] = boat.DirectThrust(state, command) ;
    else
        error('Unknown boat thruster configuration. Check Config_Boat.m')
    end
    
    % Use collected forces and moments to step the dynamics forward one dt
    StepDynamics ;
    
    % Any special events go here
    Event ;
    
    % sinc animation timing for realtime sims
    if(sim.type ~= 1)
        killTime = sim.dt - toc ;
        if(killTime > 0)
            pause(killTime) ;
            syncFlag = false ;
        else
            syncFlag = true ;
        end    
    end
end

if(sim.type == 1)
    Draw1D ;
    fprintf('\nDone !!\n') ;
end

if(sim.quit == true)
    close
    fprintf('Quitting...\n') ;
end