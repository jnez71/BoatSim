function [] = ui_robot_callback(hObject, eventdata, robot)

dlg_title = 'Configure Robot' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Position Waypoint (m)') ;
default(1) = cellstr(mat2str(round(robot.target(1:2).*100)./100)) ;

prompt(2) = cellstr('Yaw Waypoint (deg)') ;
default(2) = cellstr(mat2str(round(robot.target(3)*180/pi*100)/100)) ;

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

prompt(8) = cellstr('ka gains') ;
default(8) = cellstr(mat2str(robot.ka)) ;

prompt(9) = cellstr('kG gains') ;
default(9) = cellstr(mat2str(robot.kG(1,1))) ;

prompt(10) = cellstr('ku gains') ;
default(10) = cellstr(mat2str(robot.ku(1,1))) ;

prompt(11) = cellstr('Trajectory Type') ;
default(11) = cellstr(robot.trajectory_type) ;

prompt(12) = cellstr('Velocity Profile Maximum') ;
default(12) = cellstr(mat2str(robot.tVmax)) ;

prompt(13) = cellstr('Polar Path Radius') ;
default(13) = cellstr(mat2str(robot.tRadius)) ;

prompt(14) = cellstr('Active?') ;
default(14) = cellstr(mat2str(robot.active)) ;

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
[ka, status] = str2num(answer{8}) ;
[kG, status] = str2num(answer{9}) ;
[ku, status] = str2num(answer{10}) ;
ttype = answer{11} ;
[tVmax, status] = str2num(answer{12}) ;
[radius, status] = str2num(answer{13}) ;
[act, status] = str2num(answer{14}) ;

robot.active = act ;
if ~strcmp(robot.controller_type, ctype) || kG(1) == -1
    robot.adaptConst = [0;0;0] ;
    if strcmp(ctype, 'adapt_cl')
        robot.adaptDrag = zeros(robot.nparams,1) ;
    else
        robot.adaptDrag = zeros(5,1) ;
    end
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
if ~strcmp(robot.trajectory_type, ttype) || ~isequal(robot.target(1:2), pTar) || robot.target(3) ~= yTar*pi/180 || robot.tRadius ~= radius
    robot.target(1:2) = pTar ;
    robot.target(3) = yTar*pi/180 ;
    robot.trajectory_type = ttype ;
    robot.tRadius = radius ;
    robot.tChange = true ;
end
robot.tVmax = tVmax ;
robot.ka = ka ;
robot.ku = ku(1)*eye(robot.nparams) ;
if strcmp(robot.controller_type, 'adapt_gd')
    robot.kG = kG(1)*eye(5) ;
elseif strcmp(robot.controller_type, 'adapt_cl')
    robot.kG = kG(1)*eye(robot.nparams) ;
end

end