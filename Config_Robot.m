classdef Config_Robot < handle
% All robot decision making processes go here.
    
    properties
        memory ;
        model ;
        pDes ;
        yDes ;
    end
    
    methods
        
        function robot = Config_Robot(sim)
            robot.pDes = sim.pDes0 ;
            robot.yDes = sim.yDes0 ;
        end
        
        function command = Decide(robot,state,boat)
            
            stateMeas = robot.Sensors(state) ;
            stateEst = robot.Estimate(stateMeas) ;
            
            robot.memory = stateEst ;
            robot.model = SysID(robot) ;
            
            [FtDes,MtDes] = robot.Controller(stateEst) ;
            
            if(strcmp(boat.type, 'azi'))
                command = AziMap(robot,FtDes,MtDes,boat) ;
            else
                command = FixedMap(robot,FtDes,MtDes,boat) ;
            end
            
        end
        
        function stateMeas = Sensors(robot,state)
            
            stateMeas = state ;
            stateMeas.p = stateMeas.p + 0 ;
            
        end
        
        function stateEst = Estimate(robot,stateMeas)
            
            stateEst = stateMeas ;
            
        end
        
        function [FtDes,MtDes] = Controller(robot,stateEst)
            
            FtDes = [0,0,0]' ;
            MtDes = [0,0,0]' ;
            
        end
        
        function command = AziMap(robot,FtDes,MtDes,boat)
            
            command = [0,0,0,0] ;
            
        end
        
        function command = FixedMap(robot,FtDes,MtDes,boat)
            
            command = [0,0,0,0] ;
            
        end
        
        function model = SysID(robot)
            
            model = 0 ;
            
        end
        
    end
    
end