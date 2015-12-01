function [] = ui_quit_callback(hObject, eventdata, sim)

sim.quit = true ;
fprintf('\n') ;

end