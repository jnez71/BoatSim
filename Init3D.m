[vertsOrig, faces, normals, stlName] = stlread('PropaGator_Body.STL') ; % load boat model
vertsOrig = (0.0254 * eye(3) * vertsOrig')' ; % scale boat model to meters

figure ; % open figure window
grid on ; % draw grid lines so motion is more apparent

axis(sim.windowSize) ; % set axis limits
xlabel('X') ;
ylabel('Y') ;
zlabel('Z') ;
axis('image') ; % set uniform axis scaling for proper image display
view(sim.view0) ; % set initial viewing angle
camlight('headlight') ; % lighting

hold on ; % lock this figure as the active figure

% Other things to draw in:
water = patch([sim.windowSize(1),sim.windowSize(2),sim.windowSize(2),sim.windowSize(1)],[sim.windowSize(1),sim.windowSize(1),sim.windowSize(2),sim.windowSize(2)],'blue','FaceAlpha',0.25) ;

% Initialize ui
pbrh = 10 ;
pbh = 25 ;
pbw = 50 ;
pbps = [10 : pbw : 10*pbw] ;
ui_sim = uicontrol('String', 'sim', 'Position', [pbps(1) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_sim_callback(hObject, eventdata, sim)) ;
ui_boat = uicontrol('String', 'boat', 'Position', [pbps(2) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_boat_callback(hObject, eventdata, boat)) ;
ui_env = uicontrol('String', 'env', 'Position', [pbps(3) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_env_callback(hObject, eventdata, env)) ;
ui_robot = uicontrol('String', 'robot', 'Position', [pbps(4) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_robot_callback(hObject, eventdata, robot)) ;
ui_cam = uicontrol('String', 'cam', 'Position', [pbps(5) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_cam_callback(hObject, eventdata, sim)) ;
ui_quit = uicontrol('String', 'quit', 'Position', [pbps(6) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_quit_callback(hObject, eventdata, sim)) ;
ui_time = uicontrol('Style', 'text', 'String', 'Time:0', 'Position', [pbps(8) 1.75*pbrh pbw*3 pbh/2]) ;

Draw3D ; % draw first frame
pause(2) ; % give user some time to grab the window