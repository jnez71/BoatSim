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
        thrusters ; % AZI: [Tbl, TbR, phibl, phibr], FIXED: [Tbl, TbR, Tfl, Tfr], DIRECT: [fx, fy, Mz, 0]
        output ;
        saturated = false ; % true or false
    end
    
    methods
        function state = State(sim, n_outputs)
            state.t = 0 ;
            state.p = sim.p0 ;
            state.v = sim.v0 ;
            state.th = [0,0,sim.y0]' ;
            state.w = sim.w0 ;
            state.R = [cos(sim.y0),-sin(sim.y0),0 ; sin(sim.y0),cos(sim.y0),0 ; 0,0,1] ;
            state.heading = state.R(:,1) ;
            state.thrusters = [0, 0, 0, 0] ;
            state.output = zeros(n_outputs, 1) ;
        end
    end
    
end