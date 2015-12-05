function [] = ui_sim_callback(hObject, eventdata, sim)

dlg_title = 'Configure Simulation' ;

prompt = {} ;
default = {} ;

prompt(1) = cellstr('Timestep (s)') ;
default(1) = cellstr(mat2str(sim.dt)) ;

prompt(2) = cellstr('Iterations Per Refresh') ;
default(2) = cellstr(mat2str(sim.showFrame)) ;

prompt(3) = cellstr('Camera Origin') ;
default(3) = cellstr(mat2str(round(sim.ocam.*100)./100)) ;

prompt(4) = cellstr('X Span') ;
default(4) = cellstr(mat2str(sim.spans(1:2))) ;

prompt(5) = cellstr('Y Span') ;
default(5) = cellstr(mat2str(sim.spans(3:4))) ;

prompt(6) = cellstr('Z Span') ;
default(6) = cellstr(mat2str(sim.spans(5:6))) ;

[answer] = inputdlg(prompt, dlg_title, 1, default) ;
if isempty(answer)
    return
end

[dt, status] = str2num(answer{1}) ;
[showFrame, status] = str2num(answer{2}) ;
[ocam, status] = str2num(answer{3}) ;
[xspan, status] = str2num(answer{4}) ;
[yspan, status] = str2num(answer{5}) ;
[zspan, status] = str2num(answer{6}) ;

sim.dt = dt ;
sim.showFrame = showFrame ;
sim.ocam = ocam ;
sim.spans = [xspan, yspan, zspan] ;
sim.Set_Window() ;
axis(sim.windowSize) ;

end