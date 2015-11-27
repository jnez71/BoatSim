classdef History < handle
% Data is accumulated here for timeseries simulations. Holds the state, the time
% and 3 arbitrary outputs of interest.
    
    properties
        t ;
        p ;
        v ;
        th ;
        w ;
        output ;
        pDes ;
        yDes ;
    end
    
    methods
        
        function history = History(sim)
            history.t = [0 : sim.dt : sim.T] ;
            history.p = zeros(3,length(history.t)) ;
            history.v = zeros(3,length(history.t)) ;
            history.th = zeros(3,length(history.t)) ;
            history.w = zeros(3,length(history.t)) ;
            history.output = zeros(3,length(history.t)) ;
            history.pDes = sim.pDes0 ;
            history.yDes = sim.yDes0 ;
        end
        
        function record(history,state,robot,i)
            history.t(i) = state.t ;
            history.p(:,i) = state.p ;
            history.v(:,i) = state.v ;
            history.th(:,i) = state.th ;
            history.w(:,i) = state.w ;
            history.output(:,i) = state.output ;
            history.pDes = robot.pDes ;
            history.yDes = robot.yDes ;
        end
        
    end
    
end