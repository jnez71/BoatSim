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
        
        case 1
            sim.type = choice ;
            break ;
            
        case {2,3}
            fprintf('\nAt any time press one of these keys to configure parameters:\n') ;
            fprintf('s - simulation\n') ;
            fprintf('b - boat\n') ;
            fprintf('e - environment\n') ;
            fprintf('r - robot\n') ;
            fprintf('c - shifts camera to the boat\n') ;
            fprintf('q - quit\n') ;
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