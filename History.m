classdef History < handle
% Data is accumulated here for timeseries simulations. Holds the state, the
% timestamp, thrust commands, and 3 arbitrary outputs of interest.
    
    properties
        t ;
        p ;
        v ;
        th ;
        w ;
        output ;
        pDes ;
        yDes ;
        vDes ;
        wDes ;
        thrusters ;
    end
    
    methods
        
        function history = History(sim, n_outputs)
            history.t = [0 : sim.dt : sim.T] ;
            history.p = zeros(3,length(history.t)) ;
            history.v = zeros(3,length(history.t)) ;
            history.th = zeros(3,length(history.t)) ;
            history.w = zeros(3,length(history.t)) ;
            history.pDes = zeros(2,length(history.t)) ;
            history.yDes = zeros(1,length(history.t)) ;
            history.thrusters = zeros(4,length(history.t)) ;
            history.output = zeros(n_outputs,length(history.t)) ;
            
        end
        
        function record(history,state,robot,i)
            history.t(i) = state.t ;
            history.p(:,i) = state.p ;
            history.v(:,i) = state.v ;
            history.th(:,i) = state.th ;
            history.w(:,i) = state.w ;
            history.pDes(:,i) = robot.pDes ;
            history.yDes(:,i) = robot.yDes ;
            history.vDes(:,i) = robot.vDes ;
            history.wDes(:,i) = robot.wDes ;
            history.thrusters(:,i) = state.thrusters' ;
            history.output(:,i) = state.output ;
        end
        
    end
    
end