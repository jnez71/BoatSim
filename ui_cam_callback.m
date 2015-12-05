function [] = ui_cam_callback(hObject, eventdata, sim, state)

sim.ocam = [state.p(1), state.p(2), sim.ocam(3)] ;
sim.Set_Window() ;
axis(sim.windowSize) ;

end