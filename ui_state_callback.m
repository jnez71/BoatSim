function [] = ui_boat_callback(hObject, eventdata, state)

dlg_title = 'Configure State' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Position (m)') ;
default(1) = cellstr(mat2str(round(state.p.*100)./100)) ;

prompt(2) = cellstr('Velocity (m/s)') ;
default(2) = cellstr(mat2str(round(state.v.*100)./100)) ;

prompt(3) = cellstr('Yaw (deg)') ;
default(3) = cellstr(mat2str(round(state.th(3).*(180/pi).*100)./100)) ;

prompt(4) = cellstr('Angular Velocity (body frame, deg/s)') ;
default(4) = cellstr(mat2str(round(state.w.*(180/pi).*100)./100)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[p, status] = str2num(answer{1}) ;
[v, status] = str2num(answer{2}) ;
[yaw, status] = str2num(answer{3}) ;
[wb, status] = str2num(answer{4}) ;

state.p = p ;
state.v = v ;
% if state.heading(1) >= h(1)+0.01 || state.heading(1) <= h(1)-0.01
%     h = h./norm(h) ;
%     state.R = state.head2rot(h) ;
%     state.heading = h ;
%     state.th = [atan2(state.R(3,2),state.R(3,3)) ; atan2(-state.R(3,1),sqrt(state.R(3,2)^2+state.R(3,3)^2)) ; atan2(state.R(2,1),state.R(1,1))] ;
% end
% R = [cos(th(1)),-sin(th(1)),0;sin(th(1)),cos(th(1)),0;0,0,1]*[cos(th(2)),0,-sin(th(2));0,1,0;sin(th(2)),0,cos(th(2))]*[1,0,0;0,cos(th(3)),sin(th(3));0,-sin(th(3)),cos(th(3))] ;
if state.th(3) >= yaw+0.001 || state.th(3) <= yaw-0.001
    R = [cosd(yaw),-sind(yaw),0;sind(yaw),cosd(yaw),0;0,0,1] ;
    state.th = [0, 0, yaw*pi/180] ;
    state.R = R ;
end
state.w = state.R*(wb.*(pi/180)) ;

end