function [] = ui_env_callback(hObject, eventdata, env)

dlg_title = 'Configure Environmental Effects' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('God Force (N)') ;
default(1) = cellstr(mat2str(round(env.Fz.*100)./100)) ;

prompt(2) = cellstr('God Torque (body frame, N*m)') ;
default(2) = cellstr(mat2str(round(env.Mz.*100)./100)) ;

prompt(3) = cellstr('Translational Drag Coefficients (N/(m/s)') ;
default(3) = cellstr(mat2str(round(env.D.*100)./100)) ;

prompt(4) = cellstr('Rotational Drag Coefficients (N*m/(rad/s)') ;
default(4) = cellstr(mat2str(round(env.D_rot.*100)./100)) ;

prompt(5) = cellstr('Water Density (kg/m^3)') ;
default(5) = cellstr(mat2str(round(env.rho.*100)./100)) ;

prompt(6) = cellstr('Active?') ;
default(6) = cellstr(mat2str(env.active)) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[godforce, status] = str2num(answer{1}) ;
[godtorque, status] = str2num(answer{2}) ;
[tdrag, status] = str2num(answer{3}) ;
[rdrag, status] = str2num(answer{4}) ;
[rho, status] = str2num(answer{5}) ;
[act, status] = str2num(answer{6}) ;

env.Fz = godforce ;
env.Mz = godtorque ;
env.D = tdrag ;
env.D_rot = rdrag ;
env.rho = rho ;
env.active = act ;

end