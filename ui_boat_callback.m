function [] = ui_boat_callback(hObject, eventdata, boat, state)

dlg_title = 'Configure Boat' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Mass (kg)') ;
default(1) = cellstr(mat2str(boat.m)) ;

prompt(2) = cellstr('Yaw Inertia (kg*m^2)') ;
default(2) = cellstr(mat2str(boat.I(3,3))) ;

prompt(3) = cellstr('Thruster Layout') ;
default(3) = cellstr(boat.type) ;

prompt(4) = cellstr('Max Thrust (per thruster, N)') ;
default(4) = cellstr(mat2str(boat.maxT)) ;

prompt(5) = cellstr('Azi-Thruster Rotate Speed (deg/s)') ;
default(5) = cellstr(mat2str(round(boat.phidot.*(180/pi).*100)./100)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[mass, status] = str2num(answer{1}) ;
[inertia, status] = str2num(answer{2}) ;
type = answer{3} ;
[thrust, status] = str2num(answer{4}) ;
[phidot, status] = str2num(answer{5}) ;

boat.m = mass ;
boat.I(3,3) = inertia ;
boat.invI = inv(boat.I) ;
if (strcmp(type, 'fixed') || strcmp(type, 'azi') || strcmp(type, 'direct'))
    boat.type_next = type ;
    state.thrusters = [0,0,0,0] ;
else
    fprintf('\nInvalid thruster layout. Choose azi, fixed, or direct. \n') ;
end
boat.maxT = thrust ;
boat.phidot = phidot.*(pi/180) ;

end