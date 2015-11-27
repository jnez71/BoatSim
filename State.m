classdef State
% Current state information is stored here, as well as the current time,
% the current waypoint, and the current arbitrary outputs.
    
    properties
        t ;
        p ;
        v ;
        th ;
        w ;
        R ;
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
            state.output = [-1,-1,-1]' ;
        end
    end
    
end