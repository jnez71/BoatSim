classdef Config_Boat < handle
% Set the start-up boat parameters by editing the property values below.
% The current values are for PropaGator 2.
% Thruster dynamics are defined by the methods below.
    
    properties
        m = 42 ; % total mass (kg)
        I = [4.86,0.02,1.38;0.02,15.80,0.01;1.38,0.01,18.50] ; % inertia tensor (kg*m^2)
        %I = [4.86,0,0;0,15.80,0;0,0,18.50] ; % simplified prismatic-principle inertia tensor
        invI ; % inertia tensor inverse is computed once upon Config_Boat construction
        
        height = 0.2025 ; % z distance from C.O.M. to bottom surface of the boat (m)
        svol = [0.58,0.58,0.58] ; % submerged volume coefficients for z, roll, and pitch (m^2 or m^3/rad)
        COD = [-0.01;0;-0.9*0.2025] ; % center of drag (m)
        
        type = 'fixed' ; % thruster configuration ('azi' or 'fixed' or 'direct')
        maxT = 120 ; % maximum thrust per thruster (N)
        Lbr = [-0.5215;-0.3048;-0.0123] ; % vector from C.O.M. to back-right thruster (m)
        Lbl = [-0.5215;0.3048;-0.0123] ; % vector from C.O.M. to back-left thruster (m)
        
        % For type 'fixed' only
        Lfr = [0.5215;-0.3048;-0.0123] ; % vector from C.O.M. to front-right thruster (m)
        Lfl = [0.5215;0.3048;-0.0123] ; % vector from C.O.M. to front-left thruster (m)
        dfr = [0.7071;0.7071;0] ; % direction vector of front-right positive thrust
        dfl = [0.7071;-0.7071;0] ; % direction vector of front-left positive thrust
        dbr = [0.7071;-0.7071;0] ; % direction vector of back-right positive thrust
        dbl = [0.7071;0.7071;0] ; % direction vector of back-left positive thrust
        
        % For type 'azi' only
        phiMax = pi ; % maximum azimuthing angle from +x about +z (rad, 0 to 2*pi)
        phidot = pi ; % speed of azimuthing (rad/s)
        
        % Horrible programming that should never be seen. So many work-arounds
        type_next ;
    end
    
    methods
        function boat = Config_Boat(boat)
            boat.invI = inv(boat.I) ;
            boat.type_next = boat.type ;
        end
        
        
        function [Ft,Mt] = AziThrust(boat, state, sim, command)
            % unpack commands into desired thrusts and angles
            Cl = command(1) ;
            Cr = command(2) ;
            phil = command(3) ;
            phir = command(4) ;
            % maximum amount thruster can turn per sim step
            dphi = boat.phidot * sim.dt ;
            % servo angle errors
            phil_error = phil - state.thrusters(3) ;
            phir_error = phir - state.thrusters(4) ;
            % step dynamics for left servo
            if abs(phil_error) <= dphi
                state.thrusters(3) = phil ;
            else
                state.thrusters(3) = state.thrusters(3) + dphi*sign(phil_error) ;
            end
            % step dynamics for right servo
            if abs(phir_error) <= dphi
                state.thrusters(4) = phir ;
            else
                state.thrusters(4) = state.thrusters(4) + dphi*sign(phir_error) ;
            end
            % left thruster saturation
            if abs(Cl) <= boat.maxT
                state.thrusters(1) = Cl ;
            else
                state.thrusters(1) = boat.maxT*sign(Cl) ;
            end
            % right thruster saturation
            if abs(Cr) <= boat.maxT
                state.thrusters(2) = Cr ;
            else
                state.thrusters(2) = boat.maxT*sign(Cr) ;
            end
            % compute thrust components with current azi angles and magnitudes
            Tl = state.thrusters(1)*[cos(state.thrusters(3)), sin(state.thrusters(3)), 0]' ;
            Tr = state.thrusters(2)*[cos(state.thrusters(4)), sin(state.thrusters(4)), 0]' ;
            % convert components to wrench using thruster geometry
            Ft = state.R*(Tl + Tr) ;
            Mt = cross(boat.Lbl, Tl) + cross(boat.Lbr, Tr) ;  
        end
        
        
        function [Ft,Mt] = FixedThrust(boat, state, command)
            % pack thruster geometries into convenient matrices
            D = [boat.dbl,boat.dbr,boat.dfl,boat.dfr] ;
            L = [boat.Lbl,boat.Lbr,boat.Lfl,boat.Lfr] ;
            Ftb = [0;0;0] ;
            Mt = [0;0;0] ;
            % go through each thruster finding thrust components
            for i = [1:4]
                % thrust saturation
                if abs(command(i)) <= boat.maxT
                    state.thrusters(i) = command(i) ;
                else
                    state.thrusters(i) = boat.maxT*sign(command(i)) ;
                end
                % components
                Tb = state.thrusters(i)*D(:,i) ;
                % add this contributor to the wrench with body frame force
                Ftb = Ftb + Tb ;
                Mt = Mt + cross(L(:,i), Tb) ;
            end
            % world frame force
            Ft = state.R*Ftb ;
        end
        
        
        function [Ft,Mt] = DirectThrust(boat, state, command)
            % saturations
            command_sat = command ;
            if abs(command_sat(1)) > 250
                command_sat(1) = 250*sign(command_sat(1)) ;
            end
            if abs(command_sat(2)) > 250
                command_sat(2) = 250*sign(command_sat(2)) ;
            end
            if abs(command_sat(3)) > 400
                command_sat(3) = 400*sign(command_sat(3)) ;
            end
            % unpack limited command into thrusters
            state.thrusters = [command_sat, 0] ;
            % wrench
            Ft = [command_sat(1:2),0]' ;
            Mt = [0,0,command_sat(3)]' ;
        end
    end
end