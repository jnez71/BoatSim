classdef Config_Boat
% Set the start-up boat parameters by editing the property values below.
% The current values are for PropaGator 2.
% Thruster dynamics are defined by the methods below.
    
    properties
        m = 42 ; % total mass (kg)
        I = [4.86,0.02,1.38;0.02,15.80,0.01;1.38,0.01,18.50] ; % inertia tensor (kg*m^2)
        %I = [4.86,0,0;0,15.80,0;0,0,18.50] ; % simplified prismatic-principle inertia tensor
        %I = diag([5,5,5]) ;
        invI ; % inertia tensor inverse is computed once upon Config_Boat construction
        
        height = 0.2025 ; % z distance from C.O.M. to bottom surface of the boat (m)
        svol = [0.58,0.58,0.58] ; % submerged volume coefficients for z, roll, and pitch (m^2 or m^3/rad)
        
        type = 'fixed' ; % thruster configuration ('azi' or 'fixed')
        maxT = 120 ; % maximum thrust per thruster (N)
        Lbr = [-0.5215;-0.3048;-0.0123] ; % vector from C.O.M. to back-right thruster (m)
        Lbl = [-0.5215;0.3048;-0.0123] ; % vector from C.O.M. to back-left thruster (m)
        
        % For type 'fixed' only
        Lfr = [0.5215;-0.3048;-0.0123] ; % vector from C.O.M. to front-right thruster (m)
        Lfl = [0.5215;0.3048;-0.0123] ; % vector from C.O.M. to front-left thruster (m)
        dfr = [0.7071;0.7071;0] ; % direction vector of front-right positive thrust
        dfl = [0.7071;-0.7071;0] ; % direction vector of front-left positive thrust
        dbr = [-0.7071;0.7071;0] ; % direction vector of back-right positive thrust
        dbl = [-0.7071;-0.7071;0] ; % direction vector of back-left positive thrust
        
        % For type 'azi' only
        phiMax = pi ; % maximum azimuthing angle from +x about +z (rad, 0 to 2*pi)
        phidot = pi ; % speed of azimuthing (rad/s)
        phiR = 0 ; % initial back right azi angle (rad, 0 to phiMax)
        phiL = 0 ; % initial back left azi angle (rad, 0 to phiMax)
    end
    
    methods
        
        function boat = Config_Boat(boat)
            boat.invI = inv(boat.I) ;
        end
        
        function [Ft,Mt] = aziThrust(boat,command)
            
            Ft = [0,0,0]' ;
            Mt = [0,0,0]' ;
            
        end
        
        function [Ft,Mt] = fixedThrust(boat,command)
            
            Ft = [0,0,0]' ;
            Mt = [0,0,0]' ;
            
        end
        
    end
end