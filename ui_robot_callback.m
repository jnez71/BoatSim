function [] = ui_robot_callback(hObject, eventdata, robot)

dlg_title = 'Configure Robot' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Position Waypoint (m)') ;
default(1) = cellstr(mat2str(robot.pDes)) ;

prompt(2) = cellstr('Yaw Waypoint (deg)') ;
default(2) = cellstr(mat2str(robot.yDes*180/pi)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[pDes, status] = str2num(answer{1}) ;
[yDes, status] = str2num(answer{2}) ;

robot.pDes = pDes ;
robot.yDes = yDes*pi/180 ;

end