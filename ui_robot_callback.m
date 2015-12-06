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

prompt(4) = cellstr('kp gains') ;
default(4) = cellstr(mat2str(robot.kp)) ;

prompt(5) = cellstr('kd gains') ;
default(5) = cellstr(mat2str(robot.kd)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[pDes, status] = str2num(answer{1}) ;
[yDes, status] = str2num(answer{2}) ;
[act, status] = str2num(answer{3}) ;
[kp, status] = str2num(answer{4}) ;
[kd, status] = str2num(answer{5}) ;

robot.pDes = pDes ;
robot.yDes = yDes*pi/180 ;
robot.active = act ;
robot.kp = kp ;
robot.kd = kd ;

end