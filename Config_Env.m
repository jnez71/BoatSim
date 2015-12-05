classdef Config_Env < handle
% Set the start-up environment parameters by editing the property values below.
% Edit the methods to change the environmental force models. All environment
% methods must output a force vector [Fx,Fy,Fz]' expressed in WORLDFRAME coordinates
% and a moment vector [M_roll,M_pitch,M_yaw]' expressed about BODYFRAME axes.
    
    properties
        % gravity
        g = 9.81 ; % gravitational field strength (m/s^2)
        % buoyancy
        rho = 1000 ; % density of water (kg/m^3)
        % drag
        D = [8.8,213,800] ; % drag coefficients on translations (N/(m/s))
        D_rot = [93,500,200] ; % drag coefficients on rotations (N*m/(rad/s))
        vPlane = 5.14 ; % bodyframe x velocity necessary to start hydroplaning (m/s)
        PlaneFrac = 0.95 ; % fraction of forward drag coefficient while hydroplaning (N/(m/s)^2)
        % wind
        wd0 = [0,0] ; % initial wind force vector (N)
        wd_r = 0 ; % wind-induced rotations factor
        wd_c = 0 ; % wind chaos factor
        % waves
        wv0 = [0,0] ; % initial wave direction vector
        wvMag_mu = 0 ; % mean wave strength (N)
        wvMag_sig = 0 ; % standard deviation of wave strength (N)
        wvPer_mu = 0 ; % mean wave generation period (s)
        wvPer_sig = 0 ; % standard deviation of wave generation period (s)
        % disturbance
        Fz = [0,0,0]' ; % initial Zanza force (N)
        Mz = [0,0,0]' ; % initial Zanza moment (N*m)
        % general
        active = true ; % should environment affect the body
    end
    
    methods
        
        % flat-world uniform gravity model
        function [Fg,Mg] = Gravity(env,boat,state)
            
            Fg = [0,0,-(boat.m)*(env.g)]' ;
            Mg = [0,0,0]' ;
            
        end
        
        % small angles hydrostatic buoyancy model
        function [Fb,Mb] = Buoyancy(env,boat,state)
            
            Fbz = env.rho * boat.svol(1) * (boat.height - state.p(3)) * env.g ;
            if(Fbz < 0)
                Fb = [0,0,0]' ;
            else
                Fb = [0,0,Fbz]' ;
            end
            
            if(state.p(3)>boat.height*1.25)
                Mb = [0,0,0]' ;
            else
                Mb_roll = -env.rho * boat.svol(2) * state.th(1) * env.g ;
                Mb_pitch = -env.rho * boat.svol(3) * state.th(2) * env.g ;
                Mb = [Mb_roll,Mb_pitch,0]' ;
            end
            
        end
        
        % switch-mode v^2 drag model with damping on rotations and z translation
        function [Fd,Md] = Drag(env,boat,state)
            
            vBoat = state.R' * state.v ;
            Fd(3,1) = -env.D(3) * vBoat(3) ;
            if(state.p(3)>boat.height*1.1)
                Fd = [0;0;0] ;
            end
            Fd(1:2,1) = -env.D(1:2)' .* vBoat(1:2).^2 .* sign(vBoat(1:2)) ;
            
            if(vBoat(1) >= env.vPlane)
                Fd(1) = env.PlaneFrac * Fd(1) ;
            end
            
            if state.p(3) < 0
                if abs(state.p(3)) > boat.height
                    depthscale = 1 + boat.height ;
                else
                    depthscale = 1 - state.p(3) ;
                end
            else
                depthscale = 1 ;
            end
            
            Fd = depthscale * state.R * Fd ;
            Md = depthscale * -env.D_rot' .* (state.R' * state.w) ;
            
        end
        
        % slowly varying uniform wind model
        function [Fw,Mw] = Wind(env,boat,state)
            
            Fw = [0,0,0]' ;%%%tbi%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Mw = [0,0,0]' ;
            
        end
        
        % wide-line waves model
        function [Fv,Mv] = Waves(env,boat,state)
            
            Fv = [0,0,0]' ;%%%%%%%tbi%%%%%%%%%%%%%%%%%%%%%%%%5
            Mv = [0,0,0]' ;
            
        end
        
    end
    
end