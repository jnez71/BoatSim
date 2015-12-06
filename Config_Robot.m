classdef Config_Robot < handle
% All robot decision making processes go here.
    
    properties
        memory ;
        model ;
        pDes ;
        yDes ;
        active = true ;
        kp = [50; 50; 120] ;
        kd = [50; 50; 120] ;
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
            elseif(strcmp(boat.type, 'fixed'))
                command = FixedMap(robot,FtDes,MtDes,boat) ;
            elseif(strcmp(boat.type, 'direct'))
                command = [FtDes', MtDes] ;
            end
            
        end
        
        function stateMeas = Sensors(robot,state)
            
            % STATE IS A HANDLE SO IT PASSES BY REFERENCE - BE CAREFUL
            stateMeas = state ;
            
        end
        
        function stateEst = Estimate(robot,stateMeas)
            
            stateEst = stateMeas ;
            
        end
        
        function [FtDes,MtDes] = Controller(robot,stateEst)
            
            FtDes = robot.kp(1:2).*(robot.pDes(1:2) - stateEst.p(1:2)) + robot.kd(1:2).*(-stateEst.v(1:2)) ;
            
            MtDes = robot.kp(3).*(robot.yDes - stateEst.th(3)) ;
            
        end
        
        function command = AziMap(robot,FtDes,MtDes,boat)
            
            % [Tbl, TbR, phibl, phibr]
            command = [0,0,0,0] ;
            
        end
        
        function command = FixedMap(robot,FtDes,MtDes,boat)
            
            % [Tbl, TbR, Tfl, Tfr]
            command = [0,0,0,0] ;
            
        end
        
        function model = SysID(robot)
            
            model = 0 ;
            
        end
        
    end
    
end