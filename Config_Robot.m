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
        controller_type = 'adapt_gd' ; % 'pid', 'pd', 'adapt_gd', or 'adapt_cl'
        trajectory_type = 'circle' ; % 'waypoint', 'arc'(TBI), 'circle', or 'polar' (TBI)
        tChange = false ;
        tVmax = [5; 0.7; 1.5] ; % m/s, m/s, rad/s
        tRadius = 8 ; % m
        fullDist ;
        kp = [200; 200; 280] ;
        kd = [200; 100; 100] ;
        ki = [3; 3; 3] ;
        kf = [1; 1; 1] ;
        ka = [1; 1; 1] ;
        kG = 3 ; % distributed to all
        ku = 0.0001 ; % distributed to all
        LS = false ;
        stackSize = 5 ;
        adaptConst = [0; 0; 0] ;
        adaptDrag = 'zero' ; % [d1 d2 Lc1 Lc2 Lr] (found [30; 100; -2; -10; 100]) or [nparams...] or 'zero' starts with zeros of correct size
        adaptErr ;
        Y4integ ;
        uscript ;
        Yscript ;
        nparams ;
        stack ;
        stackMin = 0 ;
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
            if strcmp(robot.controller_type, 'adapt_cl')
                robot.nparams = 16 ;
            else
                robot.nparams = 5 ;
            end
            robot.kG = robot.kG*eye(robot.nparams) ;
            robot.ku = robot.ku*eye(robot.nparams) ;
            if strcmp(robot.adaptDrag, 'zero')
                robot.adaptDrag = zeros(robot.nparams,1) ;
            end
            robot.adaptErr = zeros(robot.nparams,1) ;
            robot.Y4integ = zeros(3,robot.nparams) ;
            robot.uscript = zeros(3,1) ;
            robot.Yscript = zeros(3,robot.nparams) ;
            robot.stack = cell(robot.stackSize,2) ; % {Yscript, uscript;...}
            for i = [1:robot.stackSize]
                robot.stack(i,:) = {zeros(3,robot.nparams), zeros(3,1)} ;
            end
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
            elseif strcmp(robot.controller_type, 'adapt_gd')
                [FtDes,MtDes] = robot.Controller_AdaptGD(stateEst, boat) ;
            elseif strcmp(robot.controller_type, 'adapt_cl')
                [FtDes,MtDes] = robot.Controller_AdaptCL(stateEst, boat) ;
            else
                disp('INVALID CONTROLLER TYPE') ;
            end
            % use applicable mapper to get command
            if(strcmp(boat.type, 'azi'))
                command = AziMap(robot, FtDes, MtDes, stateEst.R, boat) ;
            elseif(strcmp(boat.type, 'fixed'))
                command = FixedMap(robot, FtDes, MtDes, stateEst.R, boat) ;
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
            elseif strcmp(robot.trajectory_type, 'waypoint')
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
                % Virtual wrench saturation, currently using fixed mapper
                B_trans = [boat.dbl, boat.dbr, boat.dfl, boat.dfr] ;
                B_rot = [cross(boat.Lbl, boat.dbl), cross(boat.Lbr, boat.dbr), cross(boat.Lfl, boat.dfl), cross(boat.Lfr, boat.dfr)] ;
                B_world = [RDes * B_trans ; RDes * B_rot] ;
                B_world = [B_world(1:2, :); B_world(6, :)] ;
                wrench = B_world * robot.FixedMap(wrench(1:2), wrench(3), RDes, boat)' ;
                % Virtual environment based on desired max velocities
                B_body = [B_trans ; B_rot] ;
                FxMax_body = B_body * boat.maxT * ones(4,1) ;
                FyMax_body = B_body * boat.maxT * [1; -1; -1; 1] ;
                MzMax_body = B_body * boat.maxT * [-1; 1; -1; 1] ;
                D_body = abs([FxMax_body(1);FyMax_body(2);MzMax_body(6)]) ./ robot.tVmax.^2 ;
                vwDes_body = RDes' * [robot.vDes; robot.wDes] ;
                dragDes_body = D_body .* vwDes_body .* abs(vwDes_body) ;
                dragDes = RDes * dragDes_body ;
                % Step virtual controller
                robot.aDes = (wrench(1:2) - dragDes(1:2)) / boat.m ;
                robot.aaDes = (wrench(3) - dragDes(3)) / boat.I(3,3) ;
                robot.pDes = robot.pDes + robot.vDes*robot.timeStep ;
                robot.yDes = robot.yDes + robot.wDes*robot.timeStep ;
                robot.vDes = robot.vDes + robot.aDes*robot.timeStep ;
                robot.wDes = robot.wDes + robot.aaDes*robot.timeStep ;
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
                robot.target = [robot.pDes ; robot.yDes] ;
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
            kpW = stateEst.R*diag(robot.kp)*stateEst.R' ;
            kdW = stateEst.R*diag(robot.kd)*stateEst.R' ;
            % Errors
            E = [robot.pDes(1:2) - stateEst.p(1:2) ; robot.YawError(stateEst.R, robot.yDes)] ;
            Edot = [robot.vDes(1:2) - stateEst.v(1:2) ; robot.wDes - stateEst.w(3)] ;
            % Acceleration feedforward
            aforward = robot.kf.*([boat.m; boat.m; boat.I(3,3)].*[robot.aDes; robot.aaDes]) ;
            % PID wrench
            wrench = kpW*E + kdW*Edot + aforward + robot.adaptConst ;
            FtDes = wrench(1:2) ;
            MtDes = wrench(3) ;
            % Update integrator value for next call
            robot.adaptConst = robot.adaptConst + (kiW .* E .* robot.timeStep) ;
        end
        
        
        function [FtDes,MtDes] = Controller_AdaptGD(robot, stateEst, boat)
            % World frame gains
            kpW = stateEst.R*diag(robot.kp)*stateEst.R' ;
            kdW = stateEst.R*diag(robot.kd)*stateEst.R' ;
            kaW = stateEst.R*diag(robot.ka)*stateEst.R' ;
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
            wrench = kpW*E + kdW*Edot + aforward + Y*robot.adaptDrag ;%+ robot.adaptConst ;
            FtDes = wrench(1:2) ;
            MtDes = wrench(3) ;
            % Update adaptor values for next call
            robot.adaptDrag = robot.adaptDrag + robot.kG*Y'*(Edot + kaW*E).*robot.timeStep ;
            %robot.adaptConst = robot.adaptConst + (robot.ki .* E .* robot.timeStep) ;
        end
        
        
        function [FtDes,MtDes] = Controller_AdaptCL(robot, stateEst, boat)
            % World frame gains
            kpW = stateEst.R*diag(robot.kp)*stateEst.R' ;
            kdW = stateEst.R*diag(robot.kd)*stateEst.R' ;
            
            % Errors
            error = [robot.pDes(1:2) - stateEst.p(1:2) ; robot.YawError(stateEst.R, robot.yDes)] ;
            errordot = [robot.vDes(1:2) - stateEst.v(1:2) ; robot.wDes - stateEst.w(3)] ;
            
            % Definitions from paper
            alpha = robot.ka(1) ;
            Psi = stateEst.th(3) ;
            c = cos(Psi) ;
            s = sin(Psi) ;
            N = stateEst.p(1) ;
            E = stateEst.p(2) ;
            nu = stateEst.R'*([stateEst.v(1:2); stateEst.w(3)]) ;
            u = nu(1) ;
            v = nu(2) ;
            r = nu(3) ;
            
            Psi_d = robot.yDes ;
            c_d = cos(Psi_d) ;
            s_d = sin(Psi_d) ;
            N_d = robot.pDes(1) ;
            E_d = robot.pDes(2) ;
            nu_d = stateEst.R'*([robot.vDes(1:2); robot.wDes]) ;
            u_d = nu_d(1) ;
            v_d = nu_d(2) ;
            r_d = nu_d(3) ;
            nud_d = stateEst.R'*([robot.aDes(1:2); robot.aaDes]) ;
            ud_d = nud_d(1) ;
            vd_d = nud_d(2) ;
            rd_d = nud_d(3) ;
            
            % Regressors
            Y2 = [(c^2*r*(c^2 + s^2) + r*s^2*(c^2 + s^2))*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) - (c^2 + s^2)*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) - (c*v + s*u)*(r_d - alpha*(Psi - Psi_d)), c^2*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) + s*u*(r_d - alpha*(Psi - Psi_d)) - c*s*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d) - c^2*r*(c^2 + s^2)*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) - c*r*s*(c^2 + s^2)*(alpha*(N - N_d) - c_d*u_d + s_d*v_d), s^2*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) + c*v*(r_d - alpha*(Psi - Psi_d)) + c*s*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d) - r*s^2*(c^2 + s^2)*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) + c*r*s*(c^2 + s^2)*(alpha*(N - N_d) - c_d*u_d + s_d*v_d),                                                                                                                                                                                                                                                        - s*(rd_d - alpha*(r - r_d)) - c*r*(r_d - alpha*(Psi - Psi_d)),                                                                                                                                                                                                                                                          s*(rd_d - alpha*(r - r_d)) + c*r*(r_d - alpha*(Psi - Psi_d)),                      0,                      0, -c*u*abs(u)*(c^2 + s^2),  s*v*abs(v)*(c^2 + s^2),  s*v*abs(r)*(c^2 + s^2),  r*s*abs(v),  r*s*abs(r),                     0,                     0,         0,         0;
                  (c*u - s*v)*(r_d - alpha*(Psi - Psi_d)) + (c^2 + s^2)*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d) + (c^2*r*(c^2 + s^2) + r*s^2*(c^2 + s^2))*(alpha*(N - N_d) - c_d*u_d + s_d*v_d), c*s*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) - c*u*(r_d - alpha*(Psi - Psi_d)) - s^2*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d) - r*s^2*(c^2 + s^2)*(alpha*(N - N_d) - c_d*u_d + s_d*v_d) - c*r*s*(c^2 + s^2)*(c_d*v_d - alpha*(E - E_d) + s_d*u_d), s*v*(r_d - alpha*(Psi - Psi_d)) - c^2*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d) - c*s*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) - c^2*r*(c^2 + s^2)*(alpha*(N - N_d) - c_d*u_d + s_d*v_d) + c*r*s*(c^2 + s^2)*(c_d*v_d - alpha*(E - E_d) + s_d*u_d),                                                                                                                                                                                                                                                          c*(rd_d - alpha*(r - r_d)) - r*s*(r_d - alpha*(Psi - Psi_d)),                                                                                                                                                                                                                                                          r*s*(r_d - alpha*(Psi - Psi_d)) - c*(rd_d - alpha*(r - r_d)),                      0,                      0, -s*u*abs(u)*(c^2 + s^2), -c*v*abs(v)*(c^2 + s^2), -c*v*abs(r)*(c^2 + s^2), -c*r*abs(v), -c*r*abs(r),                     0,                     0,         0,         0;
                  -(c*u - s*v)*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) - (c*v + s*u)*(alpha*(N - N_d) - c_d*u_d + s_d*v_d),                                                                                                                                                                                                                                                               c*u*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) + s*u*(alpha*(N - N_d) - c_d*u_d + s_d*v_d),                                                                                                                                                                                                                                                               c*v*(alpha*(N - N_d) - c_d*u_d + s_d*v_d) - s*v*(c_d*v_d - alpha*(E - E_d) + s_d*u_d), c*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d) + s*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) + s*(r - r*(c^2 + s^2))*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) - c*(r - r*(c^2 + s^2))*(alpha*(N - N_d) - c_d*u_d + s_d*v_d), c*(r - r*(c^2 + s^2))*(alpha*(N - N_d) - c_d*u_d + s_d*v_d) - s*(alpha*(c*u - c_d*u_d - s*v + s_d*v_d) - c_d*ud_d + s_d*vd_d + c_d*r_d*v_d + r_d*s_d*u_d) - s*(r - r*(c^2 + s^2))*(c_d*v_d - alpha*(E - E_d) + s_d*u_d) - c*(c_d*vd_d - alpha*(c*v - c_d*v_d + s*u - s_d*u_d) + s_d*ud_d + c_d*r_d*u_d - r_d*s_d*v_d), rd_d - alpha*(r - r_d), alpha*(r - r_d) - rd_d,                       0,                       0,                       0,           0,           0, -v*abs(v)*(c^2 + s^2), -v*abs(r)*(c^2 + s^2), -r*abs(v), -r*abs(r)];
            
            Y3 = [c*u - s*v, -c*u*(c^2 + s^2),  s*v*(c^2 + s^2),          -r*s,            r*s, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                  c*v + s*u, -s*u*(c^2 + s^2), -c*v*(c^2 + s^2),           c*r,           -c*r, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                  0,                0,                0, v*(c^2 + s^2), -v*(c^2 + s^2), r, -r, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            
            Y4 = [0, -r*s*u*(c^2 + s^2 - 1), -c*r*v*(c^2 + s^2 - 1),               0,                0, 0, 0, -c*u*abs(u)*(c^2 + s^2),  s*v*abs(v)*(c^2 + s^2),  s*v*abs(r)*(c^2 + s^2),  r*s*abs(v),  r*s*abs(r),                     0,                     0,         0,         0;
                  0,  c*r*u*(c^2 + s^2 - 1), -r*s*v*(c^2 + s^2 - 1),               0,                0, 0, 0, -s*u*abs(u)*(c^2 + s^2), -c*v*abs(v)*(c^2 + s^2), -c*v*abs(r)*(c^2 + s^2), -c*r*abs(v), -c*r*abs(r),                     0,                     0,         0,         0;
                  0,        u*v*(c^2 + s^2),       -u*v*(c^2 + s^2), r*u*(c^2 + s^2), -r*u*(c^2 + s^2), 0, 0,                       0,                       0,                       0,           0,           0, -v*abs(v)*(c^2 + s^2), -v*abs(r)*(c^2 + s^2), -r*abs(v), -r*abs(r)];
            
            % PID+adapt wrench
            wrench = kpW*error + kdW*errordot + Y2*robot.adaptDrag ;
            FtDes = wrench(1:2) ;
            MtDes = wrench(3) ;
            
            % Learning
            if robot.LS
                robot.kG = robot.kG - (robot.kG*robot.ku*robot.Yscript'*robot.Yscript*robot.kG)*robot.timeStep ;
            end
            
            robot.adaptDrag = robot.adaptDrag + robot.kG*(Y2'*(errordot + alpha*error) + robot.ku*robot.adaptErr)*robot.timeStep ;
            
            robot.uscript = robot.uscript + wrench*robot.timeStep ;
            robot.Yscript = Y3 + robot.Y4integ ;
            robot.Y4integ = robot.Y4integ + Y4*robot.timeStep ;
            
            if robot.stackSize
                svdMins = zeros(robot.stackSize,1) ;
                newDat = {robot.Yscript, robot.uscript} ;

                for i = [1:robot.stackSize]
                    cand = robot.stack ;
                    cand(i,:) = newDat ;
                    YiYi = zeros(robot.nparams) ;
                    for j = [1:robot.stackSize]
                        YiYi = YiYi + cand{j,1}'*cand{j,1} ;
                    end
                    try
                        svdMins(i) = min(svd(YiYi)) ;
                    catch
                        svdMins(i) = 0 ;
                        disp('Adaptation diverged!')
                    end
                end

                [hotsvd, hotseat] = max(svdMins);
                if hotsvd > robot.stackMin && stateEst.saturated == false
                    robot.stack(hotseat,:) = newDat ;
                    robot.stackMin = hotsvd ;
%                     learned_at_time = stateEst.t
                end

                robot.adaptErr = zeros(robot.nparams,1) ;
                for i = [1:robot.stackSize]
                    robot.adaptErr = robot.adaptErr + robot.stack{i,1}'*(robot.stack{i,2} - robot.stack{i,1}*robot.adaptDrag) ;
                end
                
            else
                robot.adaptErr = robot.Yscript'*(robot.uscript - robot.Yscript*robot.adaptDrag) ;
            end
        end
        
        
        function command = AziMap(robot, FtDes, MtDes, stateEst, boat)
            % [Tbl, Tbr, phibl, phibr]
            command = [-80,11000,pi/3,-pi] ;%(testwrench)%%TBI%%%%%%%%%%%
        end
        
        
        function command = FixedMap(robot, FtDes, MtDes, R, boat)
            % [Tbl, Tbr, Tfl, Tfr]
            B_trans = [boat.dbl, boat.dbr, boat.dfl, boat.dfr] ;
            B_rot = [cross(boat.Lbl, boat.dbl), cross(boat.Lbr, boat.dbr), cross(boat.Lfl, boat.dfl), cross(boat.Lfr, boat.dfr)] ;
            B_world = [R * B_trans ; R * B_rot] ;
            B_world = [B_world(1:2, :); B_world(6, :)] ;
            wrench = [FtDes; MtDes] ;
            % B * command' = wrench
            command = (pinv(B_world) * wrench)' ;
            % if command is not attainable, scale back linearly
            command_max = max(abs(command)) ;
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