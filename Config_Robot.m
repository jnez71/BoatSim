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
        controller_type = 'adaptive' ; % 'pid', 'pd', or 'adaptive'
        trajectory_type = 'circle' ; % 'linear', 'arc'(TBI), 'circle', or 'polar' (TBI)
        tChange = false ;
        tVmax = [3; 0.1; 1] ; % m/s, m/s, rad/s
        tRadius = 8 ; % m
        fullDist ;
        kp = [150; 150; 200] ;
        kd = [150; 150; 120] ;
        ki = [3; 3; 3] ;
        kf = [1; 1; 1] ;
        kG = 3*diag([1; 1; 1; 1; 1]) ;
        ka = [1; 1; 1] ;
        adaptConst = [0; 0; 0] ;
        adaptDrag = [25; 50; 20; 0; 50] ;
        timeStep ;
    end
    
    methods
        function robot = Config_Robot(sim)
            robot.pDes = sim.p0(1:2) ;
            robot.yDes = sim.y0 ;
            robot.target = [sim.pDes0; sim.yDes0] ;
            robot.fullDist = norm(robot.target(1:2)-sim.p0(1:2)) ;
            robot.vDes = [0;0] ;
            robot.aDes = [0;0] ;
            robot.wDes = 0 ;
            robot.aaDes = 0 ;
            robot.timeStep = sim.dt ;
        end
        
        
        function command = Decide(robot, state, boat)
            % read sensors
            stateMeas = robot.Sensors(state) ;
            % estimate state from sensor measurements
            stateEst = robot.Estimate(stateMeas) ;
            % update stateDes with what tgen says it should be right now
            robot.TrajGen(stateEst, boat) ;
            % use chosen controller to get wrench
            if strcmp(robot.controller_type, 'pid') || strcmp(robot.controller_type, 'pd')
                [FtDes,MtDes] = robot.Controller_PID(stateEst, boat) ;
            elseif strcmp(robot.controller_type, 'adaptive')
                [FtDes,MtDes] = robot.Controller_Adapt(stateEst, boat) ;
            else
                disp('INVALID CONTROLLER TYPE') ;
            end
            % use applicable mapper to get command
            if(strcmp(boat.type, 'azi'))
                command = AziMap(robot, FtDes, MtDes, stateEst, boat) ;
            elseif(strcmp(boat.type, 'fixed'))
                command = FixedMap(robot, FtDes, MtDes, stateEst, boat) ;
            elseif(strcmp(boat.type, 'direct'))
                command = [FtDes', MtDes] ;
            else
                disp('INVALID TRAJECTORY TYPE') ;
            end
        end
        
        
        function stateMeas = Sensors(robot, state) 
            % STATE IS A HANDLE SO IT PASSES BY REFERENCE - BE CAREFUL
            % Pick off states
            % Add noise
            stateMeas = state ;%%%TBI%%%%%%%%%%%
        end
        
        
        function stateEst = Estimate(robot, stateMeas) 
            % Estimate full state from measurements
            stateEst = stateMeas ;%%%TBI%%%%%%%%%%%
        end
        
        
        function [] = TrajGen(robot,stateEst, boat)
            if robot.tChange
                % Reset starting point to current state
                robot.pDes = stateEst.p(1:2) ;
                robot.yDes = stateEst.th(3) ;
                robot.vDes = stateEst.v(1:2) ;
                robot.wDes = stateEst.w(3) ;
                robot.aDes = [0;0] ;
                robot.aaDes = 0 ;
                robot.fullDist = norm(robot.target(1:2)-stateEst.p(1:2)) ;
                robot.tChange = false ;
            elseif strcmp(robot.trajectory_type, 'linear')
                % Simulate ideal-environment PD controller having xDes track target
                RDes = [cos(robot.yDes), -sin(robot.yDes), 0; sin(robot.yDes), cos(robot.yDes), 0; 0, 0, 1] ;
                kpW = diag(RDes*diag(robot.kp)*RDes') ;
                kdW = diag(RDes*diag(robot.kd)*RDes') ;
                E = robot.target(1:2) - robot.pDes ;
                % "smart yaw" (continuous pointnshoot) will one day be replaced with hermitian spline
                if norm(E) <= 2
                    E(3) = robot.YawError(RDes, robot.target(3)) ;
                else
                    E(3) = robot.YawError(RDes, angle(E(1)+1i*E(2))) ;
                end
                Edot = [-robot.vDes ; -robot.wDes] ;
                wrench = kpW.*E + kdW.*Edot ;
                % Virtual wrench saturation, eventually will use boat model and mapper
                if abs(wrench(1)) > 250/2
                    wrench(1) = 250/2*sign(wrench(1)) ;
                end
                if abs(wrench(2)) > 250/2
                    wrench(2) = 250/2*sign(wrench(2)) ;
                end
                if abs(wrench(3)) > 400/2
                    wrench(3) = 400/2*sign(wrench(3)) ;
                end
                % Step virtual controller
                robot.aDes = wrench(1:2)/boat.m ;
                robot.aaDes = wrench(3)/boat.I(3,3) ;
                robot.pDes = robot.pDes + robot.vDes*robot.timeStep ;
                robot.yDes = robot.yDes + robot.wDes*robot.timeStep ;
                robot.vDes = robot.vDes + robot.aDes*robot.timeStep ;
                robot.wDes = robot.wDes + robot.aaDes*robot.timeStep ;
                % Virtual imposed velocity maxima
                tVmaxW = abs(RDes*robot.tVmax) ;
                for idx = [1:2]
                    if abs(robot.vDes(idx)) > tVmaxW(idx)
                        robot.vDes(idx) = tVmaxW(idx)*sign(robot.vDes(idx)) ;
                        robot.aDes = [0;0] ;
                    end
                end
                if abs(robot.wDes) > tVmaxW(3)
                    robot.wDes = tVmaxW(3)*sign(robot.wDes) ;
                    robot.aaDes = 0 ;
                end
            elseif strcmp(robot.trajectory_type, 'arc')
                disp('traj TBI') %%%TBI
            elseif strcmp(robot.trajectory_type, 'circle')
                % Simple parametric circle function of time
                time = stateEst.t ;
                r = robot.tRadius ;
                f = 2*pi/15 ;
                robot.pDes = [r*cos(f*time); r*sin(f*time)] ;
                robot.yDes = mod(f*time+pi/2, 2*pi) ;
                if robot.yDes > pi
                    robot.yDes = robot.yDes - 2*pi ;
                end
                robot.vDes = [-r*f*sin(f*time); r*f*cos(f*time)] ;
                robot.wDes = f ;
                robot.aDes = [-r*f^2*cos(f*time); -r*f^2*sin(f*time)] ;
                robot.aaDes = 0 ;
            elseif strcmp(robot.trajectory_type, 'polar')
                disp('traj TBI') %%%TBI
            else
                error('Invalid trajectory type!') ;
            end
        end
        
        
        function [FtDes,MtDes] = Controller_PID(robot, stateEst, boat)
            if strcmp(robot.controller_type, 'pd')
                kiW = [0;0;0] ;
            else
                kiW = robot.ki ;
            end
            % World frame gains
            kpW = diag(stateEst.R*diag(robot.kp)*stateEst.R') ;
            kdW = diag(stateEst.R*diag(robot.kd)*stateEst.R') ;
            % Errors
            E = [robot.pDes(1:2) - stateEst.p(1:2) ; robot.YawError(stateEst.R, robot.yDes)] ;
            Edot = [robot.vDes(1:2) - stateEst.v(1:2) ; robot.wDes - stateEst.w(3)] ;
            % Acceleration feedforward
            aforward = robot.kf.*([boat.m; boat.m; boat.I(3,3)].*[robot.aDes; robot.aaDes]) ;
            % PID wrench
            wrench = kpW.*E + kdW.*Edot + aforward + robot.adaptConst ;
            FtDes = wrench(1:2) ;
            MtDes = wrench(3) ;
            % Update integrator value for next call
            robot.adaptConst = robot.adaptConst + (kiW .* E .* robot.timeStep) ;
        end
        
        
        function [FtDes,MtDes] = Controller_Adapt(robot, stateEst, boat)
            % World frame gains
            kpW = diag(stateEst.R*diag(robot.kp)*stateEst.R') ;
            kdW = diag(stateEst.R*diag(robot.kd)*stateEst.R') ;
            kaW = diag(stateEst.R*diag(robot.ka)*stateEst.R') ;
            % Errors
            E = [robot.pDes(1:2) - stateEst.p(1:2) ; robot.YawError(stateEst.R, robot.yDes)] ;
            Edot = [robot.vDes(1:2) - stateEst.v(1:2) ; robot.wDes - stateEst.w(3)] ;
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
            robot.adaptDrag = robot.adaptDrag + robot.kG*Y'*(Edot + kaW.*E).*robot.timeStep ;
            %robot.adaptConst = robot.adaptConst + (robot.ki .* E .* robot.timeStep) ;
        end
        
        
        function command = AziMap(robot, FtDes, MtDes, stateEst, boat)
            % [Tbl, Tbr, phibl, phibr]
            command = [-80,11000,pi/3,-pi] ;%(testwrench)%%TBI%%%%%%%%%%%
        end
        
        
        function command = FixedMap(robot, FtDes, MtDes, stateEst, boat)
            % [Tbl, Tbr, Tfl, Tfr]
            B_trans = [boat.dbl, boat.dbr, boat.dfl, boat.dfr] ;
            B_rot = [cross(boat.Lbl, boat.dbl), cross(boat.Lbr, boat.dbr), cross(boat.Lfl, boat.dfl), cross(boat.Lfr, boat.dfr)] ;
            B_world = [stateEst.R * B_trans ; stateEst.R * B_rot] ;
            B_world = [B_world(1:2, :); B_world(6, :)] ;
            wrench = [FtDes; MtDes] ;
            % B * command' = wrench
            command = (pinv(B_world) * wrench)' ;
            % if command is not attainable, scale back linearly
            command_max = max(command) ;
            if command_max > boat.maxT
                command = (boat.maxT / command_max) * command ;
            end
            % compute wrench error
            %wrench_error = wrench - B_world*command' ;
            %if norm(wrench_error) > 0.01 % some tolerance
            %   wrench_error % display error
            %end
        end
        
        
        function yErr = YawError(robot, Rcur, yDes)
            % Compute minimal yaw error by staying on SO3
            RDes = [cos(yDes),-sin(yDes),0 ; sin(yDes),cos(yDes),0 ; 0,0,1] ;
            RErr = RDes * Rcur' ;
            yErr = atan2(RErr(2,1),RErr(1,1)) ;
        end
    end
end