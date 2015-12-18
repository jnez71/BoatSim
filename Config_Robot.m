classdef Config_Robot < handle
% All robot decision making processes go here. DIY!
% See README.
    
    properties
        active = true ;
        target ;
        pDes ;
        vDes ;
        aDes ;
        yDes ;
        wDes ;
        aaDes ;
        controller_type = 'adaptive' ; % 'pid' or 'adaptive'
        trajectory_type = 'circle' ; % 'waypoint', 'circle', 'polar', or 'sine'
        tchange = false ;
        tradius = 8 ; % m
        kp = [150; 150; 180] ;
        kd = [150; 150; 150] ;
        ki = [3; 3; 3] ;
        kf = [1; 1; 1] ;
        kG = 3*diag([1;1;1;1;1]) ;
        ka = [1;1;1] ;
        adaptConst = [0;0;0] ;
        adaptDrag = [0;0;0;0;0] ;
        timeStep ;
    end
    
    methods
        function robot = Config_Robot(sim)
            robot.pDes = sim.pDes0 ;
            robot.yDes = sim.yDes0 ;
            robot.target = [sim.pDes0; sim.yDes0] ;
            robot.vDes = [0;0] ;
            robot.aDes = [0;0] ;
            robot.wDes = 0 ;
            robot.aaDes = 0 ;
            robot.timeStep = sim.dt ;
        end
        
        
        function command = Decide(robot,state,boat)
            % read sensors
            stateMeas = robot.Sensors(state) ;
            % estimate state from sensor measurements
            stateEst = robot.Estimate(stateMeas) ;
            % update stateDes with what tgen says it should be right now
            robot.TrajGen(stateEst) ;
            % use chosen controller to get wrench
            if strcmp(robot.controller_type, 'pid')
                [FtDes,MtDes] = robot.Controller_PID(stateEst, boat) ;
            elseif strcmp(robot.controller_type, 'adaptive')
                [FtDes,MtDes] = robot.Controller_Adapt(stateEst, boat) ;
            else
                disp('INVALID CONTROLLER TYPE') ;
            end
            % use applicable mapper to get command
            if(strcmp(boat.type, 'azi'))
                command = AziMap(robot,FtDes,MtDes,boat) ;
            elseif(strcmp(boat.type, 'fixed'))
                command = FixedMap(robot,FtDes,MtDes,boat) ;
            elseif(strcmp(boat.type, 'direct'))
                command = [FtDes', MtDes] ;
            else
                disp('INVALID TRAJECTORY TYPE') ;
            end
        end
        
        
        function stateMeas = Sensors(robot,state) 
            % STATE IS A HANDLE SO IT PASSES BY REFERENCE - BE CAREFUL
            % Pick off states
            % Add noise
            stateMeas = state ;%%%TBI%%%%%%%%%%%
        end
        
        
        function stateEst = Estimate(robot,stateMeas) 
            % Estimate full state from measurements
            stateEst = stateMeas ;%%%TBI%%%%%%%%%%%
        end
        
        
        function [] = TrajGen(robot,stateEst)
            if strcmp(robot.trajectory_type, 'circle')
                time = stateEst.t ;
                r = robot.tradius ;
                f = 2*pi/10 ;
                robot.pDes = [r*cos(f*time); r*sin(f*time)] ;
                robot.yDes = mod(f*time+pi/2, 2*pi) ;
                if robot.yDes > pi
                    robot.yDes = robot.yDes - 2*pi ;
                end
                robot.vDes = [-r*f*sin(f*time); r*f*cos(f*time)] ;
                robot.wDes = f ;
                robot.aDes = [-r*f^2*cos(f*time); -r*f^2*sin(f*time)] ;
                robot.aaDes = 0 ;
            elseif strcmp(robot.trajectory_type, 'waypoint')
                robot.vDes = [0;0] ;
                robot.aDes = [0;0] ;
                robot.wDes = 0 ;
                robot.aaDes = 0 ;
            end
        end
        
        
        function [FtDes,MtDes] = Controller_PID(robot,stateEst, boat)
            % World frame gains
            kpW = diag(stateEst.R*diag(robot.kp)*stateEst.R') ;
            kdW = diag(stateEst.R*diag(robot.kd)*stateEst.R') ;
            % Errors
            E = [robot.pDes(1:2) - stateEst.p(1:2) ; robot.YawError(stateEst)] ;
            Edot = [robot.vDes(1:2)-stateEst.v(1:2) ; robot.wDes - stateEst.w(3)] ;
            % Acceleration feedforward
            aforward = robot.kf.*([boat.m; boat.m; boat.I(3,3)].*[robot.aDes; robot.aaDes]) ;
            % PID wrench
            wrench = kpW.*E + kdW.*Edot + aforward + robot.adaptConst ;
            FtDes = wrench(1:2) ;
            MtDes = wrench(3) ;
            % Update integrator value for next call
            robot.adaptConst = robot.adaptConst + (robot.ki .* E .* robot.timeStep) ;
        end
        
        
        function [FtDes,MtDes] = Controller_Adapt(robot, stateEst, boat)
            % World frame gains
            kpW = diag(stateEst.R*diag(robot.kp)*stateEst.R') ;
            kdW = diag(stateEst.R*diag(robot.kd)*stateEst.R') ;
            % Errors
            E = [robot.pDes(1:2) - stateEst.p(1:2) ; robot.YawError(stateEst)] ;
            Edot = [robot.vDes(1:2)-stateEst.v(1:2) ; robot.wDes - stateEst.w(3)] ;
            % Acceleration feedforward
            aforward = robot.kf.*([boat.m; boat.m; boat.I(3,3)].*[robot.aDes; robot.aaDes]) ;
            % Drag and centripetal feedforward
            Y = [...
                [         stateEst.v(1)*cos(stateEst.th(3))^2 + stateEst.v(2)*sin(stateEst.th(3))*cos(stateEst.th(3)), stateEst.v(1)/2 - (stateEst.v(1)*cos(2*stateEst.th(3)))/2 - (stateEst.v(2)*sin(2*stateEst.th(3)))/2,                                    -stateEst.w(3)*sin(stateEst.th(3)),                                      -stateEst.w(3)*cos(stateEst.th(3)),             0];...
                [ stateEst.v(2)/2 - (stateEst.v(2)*cos(2*stateEst.th(3)))/2 + (stateEst.v(1)*sin(2*stateEst.th(3)))/2,         stateEst.v(2)*cos(stateEst.th(3))^2 - stateEst.v(1)*cos(stateEst.th(3))*sin(stateEst.th(3)),                                     stateEst.w(3)*cos(stateEst.th(3)),                                      -stateEst.w(3)*sin(stateEst.th(3)),             0];...
                [                                                                                                   0,                                                                                                   0, stateEst.v(2)*cos(stateEst.th(3)) - stateEst.v(1)*sin(stateEst.th(3)), - stateEst.v(1)*cos(stateEst.th(3)) - stateEst.v(2)*sin(stateEst.th(3)), stateEst.w(3)] ] ;
            % PID+adapt wrench, currently not using any I
            wrench = kpW.*E + kdW.*Edot + aforward + Y*robot.adaptDrag ;%+ robot.adaptConst ;
            FtDes = wrench(1:2) ;
            MtDes = wrench(3) ;
            % Update adaptor values for next call
            robot.adaptDrag = robot.adaptDrag + robot.kG*Y'*(Edot + robot.ka.*E).*robot.timeStep ;
            %robot.adaptConst = robot.adaptConst + (robot.ki .* E .* robot.timeStep) ;
        end
        
        
        function command = AziMap(robot,FtDes,MtDes,boat)
            % [Tbl, Tbr, phibl, phibr]
            command = [0,0,0,0] ;%%%TBI%%%%%%%%%%%
        end
        
        
        function command = FixedMap(robot,FtDes,MtDes,boat)
            % [Tbl, Tbr, Tfl, Tfr]
            command = [0,0,0,0] ;%%%TBI%%%%%%%%%%%
        end
        
        
        function yErr = YawError(robot, stateEst)
            % Compute minimal yaw error by staying on SO3
            RDes = [cos(robot.yDes),-sin(robot.yDes),0 ; sin(robot.yDes),cos(robot.yDes),0 ; 0,0,1] ;
            RErr = RDes * stateEst.R' ;
            yErr = atan2(RErr(2,1),RErr(1,1)) ;
        end
    end
end