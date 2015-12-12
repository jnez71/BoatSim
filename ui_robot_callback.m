function [] = ui_robot_callback(hObject, eventdata, robot)

dlg_title = 'Configure Robot' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Position Waypoint (m)') ;
default(1) = cellstr(mat2str(round(robot.pDes.*100)./100)) ;

prompt(2) = cellstr('Yaw Waypoint (deg)') ;
default(2) = cellstr(mat2str(round(robot.yDes*180/pi.*100)./100)) ;

prompt(3) = cellstr('Active?') ;
default(3) = cellstr(mat2str(robot.active)) ;

prompt(4) = cellstr('Controller Type') ;
default(4) = cellstr(robot.controller_type) ;

prompt(5) = cellstr('kp gains') ;
default(5) = cellstr(mat2str(robot.kp)) ;

prompt(6) = cellstr('kd gains') ;
default(6) = cellstr(mat2str(robot.kd)) ;

prompt(7) = cellstr('ki gains') ;
default(7) = cellstr(mat2str(robot.ki)) ;

prompt(8) = cellstr('kf gains') ;
default(8) = cellstr(mat2str(robot.kf)) ;

prompt(9) = cellstr('Trajectory Type') ;
default(9) = cellstr(robot.trajectory_type) ;

prompt(10) = cellstr('Circular Path Radius') ;
default(10) = cellstr(mat2str(robot.tradius)) ;

prompt(11) = cellstr('Circular Path Period') ;
default(11) = cellstr(mat2str(robot.tperiod)) ;

prompt(12) = cellstr('kG gains') ;
default(12) = cellstr(mat2str(diag(robot.kG))) ;

prompt(13) = cellstr('ka gains') ;
default(13) = cellstr(mat2str(robot.ka)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[pDes, status] = str2num(answer{1}) ;
[yDes, status] = str2num(answer{2}) ;
[act, status] = str2num(answer{3}) ;
ctype = answer{4} ;
[kp, status] = str2num(answer{5}) ;
[kd, status] = str2num(answer{6}) ;
[ki, status] = str2num(answer{7}) ;
[kf, status] = str2num(answer{8}) ;
ttype = answer{9} ;
[radius, status] = str2num(answer{10}) ;
[period, status] = str2num(answer{11}) ;
[kG, status] = str2num(answer{12}) ;
[ka, status] = str2num(answer{13}) ;

robot.pDes = pDes ;
robot.yDes = yDes*pi/180 ;
robot.active = act ;
if ~strcmp(robot.controller_type, ctype)
    robot.adaptConst = [0;0;0] ;
    robot.adaptDrag = [0;0;0;0;0] ;
    robot.controller_type = ctype ;
end
robot.kp = kp ;
robot.kd = kd ;
if ki(1) == -1
    robot.adaptConst = [0;0;0] ;
else
    robot.ki = ki ;
end
robot.kf = kf ;
robot.trajectory_type = ttype ;
robot.tradius = radius ;
robot.tperiod = period ;
if kG(1) == -1
    robot.adaptDrag = [0;0;0;0;0] ;
else
    robot.kG = diag(kG) ;
    robot.ka = ka ;
end

end