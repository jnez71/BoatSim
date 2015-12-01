function [] = ui_boat_callback(hObject, eventdata, state)

dlg_title = 'Configure State' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Position (m)') ;
default(1) = cellstr(mat2str(round(state.p.*100)./100)) ;

prompt(2) = cellstr('Heading') ;
default(2) = cellstr(mat2str(round(state.heading.*100)./100)) ;

prompt(3) = cellstr('Velocity (m/s)') ;
default(3) = cellstr(mat2str(round(state.v.*100)./100)) ;

prompt(4) = cellstr('Angular Velocity (deg/s)') ;
default(4) = cellstr(mat2str(round(state.w.*(180/pi).*100)./100)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[p, status] = str2num(answer{1}) ;
[h, status] = str2num(answer{2}) ;
[v, status] = str2num(answer{3}) ;
[w, status] = str2num(answer{4}) ;

state.p = p ;
if state.heading(1) >= h(1)+0.01 || state.heading(1) <= h(1)-0.01
    h = h./norm(h) ;
    state.R = state.head2rot(h) ;
    state.heading = h ;
    state.th = [atan2(state.R(3,2),state.R(3,3)) ; atan2(-state.R(3,1),sqrt(state.R(3,2)^2+state.R(3,3)^2)) ; atan2(state.R(2,1),state.R(1,1))] ;
end
state.v = v ;
state.w = w.*(pi/180) ;

end