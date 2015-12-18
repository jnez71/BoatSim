function [] = ui_robot_callback(hObject, eventdata, robot)

dlg_title = 'Configure Robot' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Position Waypoint (m)') ;
default(1) = cellstr(mat2str(robot.target(1:2))) ;

prompt(2) = cellstr('Yaw Waypoint (deg)') ;
default(2) = cellstr(mat2str(robot.target(3)*180/pi)) ;

prompt(3) = cellstr('Controller Type') ;
default(3) = cellstr(robot.controller_type) ;

prompt(4) = cellstr('kp gains') ;
default(4) = cellstr(mat2str(robot.kp)) ;

prompt(5) = cellstr('kd gains') ;
default(5) = cellstr(mat2str(robot.kd)) ;

prompt(6) = cellstr('ki gains') ;
default(6) = cellstr(mat2str(robot.ki)) ;

prompt(7) = cellstr('kf gains') ;
default(7) = cellstr(mat2str(robot.kf)) ;

prompt(8) = cellstr('kG gains') ;
default(8) = cellstr(mat2str(diag(robot.kG))) ;

prompt(9) = cellstr('ka gains') ;
default(9) = cellstr(mat2str(robot.ka)) ;

prompt(10) = cellstr('Trajectory Type') ;
default(10) = cellstr(robot.trajectory_type) ;

prompt(11) = cellstr('Polar Path Radius') ;
default(11) = cellstr(mat2str(robot.tradius)) ;

prompt(12) = cellstr('Active?') ;
default(12) = cellstr(mat2str(robot.active)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[pTar, status] = str2num(answer{1}) ;
[yTar, status] = str2num(answer{2}) ;
ctype = answer{3} ;
[kp, status] = str2num(answer{4}) ;
[kd, status] = str2num(answer{5}) ;
[ki, status] = str2num(answer{6}) ;
[kf, status] = str2num(answer{7}) ;
[kG, status] = str2num(answer{8}) ;
[ka, status] = str2num(answer{9}) ;
ttype = answer{10} ;
[radius, status] = str2num(answer{11}) ;
[act, status] = str2num(answer{12}) ;

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
if ~strcmp(robot.trajectory_type, ttype) || ~isequal(robot.target(1:2), pTar) || robot.target(3) ~= yTar*pi/180 || robot.tradius ~= radius
    robot.target(1:2) = pTar ;
    robot.target(3) = yTar*pi/180 ;
    robot.trajectory_type = ttype ;
    robot.tradius = radius ;
    robot.tchange = true ;
end
if kG(1) == -1
    robot.adaptDrag = [0;0;0;0;0] ;
else
    robot.kG = diag(kG) ;
    robot.ka = ka ;
end

end