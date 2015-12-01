classdef State < handle
% Current state information is stored here, as well as the current time,
% the current waypoint, and the current arbitrary outputs.
    
    properties
        t ;
        p ;
        v ;
        th ;
        w ;
        R ;
        heading ;
        output ;
    end
    
    methods
        function state = State(sim)
            state.t = 0 ;
            state.p = sim.p0 ;
            state.v = sim.v0 ;
            state.th = [sim.y0,0,0]' ;
            state.w = sim.w0 ;
            state.R = [cos(sim.y0),-sin(sim.y0),0 ; sin(sim.y0),cos(sim.y0),0 ; 0,0,1] ;
            state.heading = state.R(:,1) ;
            state.output = [-1,-1,-1]' ;
        end
        
        function R = head2rot(self, head)
            a = [1,0,0] ;
            b = head./norm(head) ;
            cosine = dot(a, b) ;
            if cosine >= 0.99 && cosine <= 1.01
                R = eye(3) ;
            else
                axis = cross(a, b) ;
                skew_cross = [0, -axis(3), axis(1) ; axis(3), 0, -axis(1) ; -axis(2), axis(1), 0] ;
                skew_squared = skew_cross*skew_cross ;
                R = eye(3) + skew_cross + (skew_squared * ((1-cosine)/(norm(axis))^2)) ;
            end
            [R, notactuallyR] = qr(R) ;
            if det(R) < 0
                R(:,3) = -R(:,3) ;
            end
            if dot(R*head, head) < 0
                R = R' ;
            end
        end
    end
    
end