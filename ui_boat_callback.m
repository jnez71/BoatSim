function [] = ui_boat_callback(hObject, eventdata, boat)


dlg_title = 'Configure Boat' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Mass (kg)') ;
default(1) = cellstr(mat2str(boat.m)) ;

prompt(2) = cellstr('Yaw Inertia (kg*m^2)') ;
default(2) = cellstr(mat2str(boat.I(3,3))) ;

prompt(3) = cellstr('Max Thrust (per thruster, N)') ;
default(3) = cellstr(mat2str(boat.maxT)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[mass, status] = str2num(answer{1}) ;
[inertia, status] = str2num(answer{2}) ;
[thrust, status] = str2num(answer{3}) ;

boat.m = mass ;
boat.I(3,3) = inertia ;
boat.invI = inv(boat.I) ;
boat.maxT = thrust ;


end