clc ;
close all ;
clear all ;
format short ;

% Initialize simulation with start-up parameters
sim = Config_Sim() ;

% Start-up menu
fprintf('\nWELCOME TO BOATSIM !!\nEnter the number of your choice, or -1 to quit.\n') ;
while(true)
    
    choice = input('\nSimulation type:\n1. Timeseries\n2. Realtime 2D\n3. Realtime 3D\n4. Readme\n\n') ;
    if(isempty(choice))
        choice = 0 ;
    end
    
    switch choice
        
        case {1,2,3}
            fprintf('\nInitial conditions:\n') ;
            fprintf('\n---\n\n') ;
            fprintf('position (m)\n') ;
            disp(sim.p0) ;
            fprintf('velocity (m/s)\n') ;
            disp(sim.v0) ;
            fprintf('yaw (deg)\n') ;
            disp(sim.y0*180/pi) ;
            fprintf('yaw rate (deg/s)\n') ;
            disp(sim.w0*180/pi) ;
            fprintf('---\n\n') ;
            fprintf('desired position (m)\n') ;
            disp(sim.pDes0) ;
            fprintf('desired yaw (deg)\n') ;
            disp(sim.yDes0*180/pi) ;
            fprintf('---\n\n') ;
            sim.type = choice ;
            break ;
            
        case 4
            fprintf('\nLoading the readme...\n') ;
            open('README.md') ;
            fprintf('Loaded!\n') ;
            
        case -1
            fprintf('Quitting...\n') ;
            break ;
            
        otherwise
            fprintf('(invalid)\n') ; 
            
    end
    
end

% Begin simulation unless choice was quit
if(sim.type ~= -1)
    Simulate ;
end

fprintf('\n') ;