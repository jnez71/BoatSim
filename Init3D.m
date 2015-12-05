[vertsOrig, faces, normals, stlName] = stlread('PropaGator_Body.STL') ; % load boat model
vertsOrig = (0.0254 * eye(3) * vertsOrig')' ; % scale boat model to meters

figure('Name', 'BoatSim') ; % open figure window
grid on ; % draw grid lines so motion is more apparent

axis(sim.windowSize) ; % set axis limits
xlabel('X') ;
ylabel('Y') ;
zlabel('Z') ;
axis('image') ; % set uniform axis scaling for proper image display
view(sim.view0) ; % set initial viewing angle
camlight('headlight') ; % lighting

hold on ; % lock this figure as the active figure

% Initialize ui
pbrh = 10 ;
pbh = 25 ;
pbw = 50 ;
pbps = [10 : pbw : 20*pbw] ;
ui_robot = uicontrol('String', 'robot', 'Position', [pbps(1) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_robot_callback(hObject, eventdata, robot)) ;
ui_state = uicontrol('String', 'state', 'Position', [pbps(2) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_state_callback(hObject, eventdata, state)) ;
ui_boat = uicontrol('String', 'boat', 'Position', [pbps(3) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_boat_callback(hObject, eventdata, boat)) ;
ui_env = uicontrol('String', 'env', 'Position', [pbps(4) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_env_callback(hObject, eventdata, env)) ;
ui_sim = uicontrol('String', 'sim', 'Position', [pbps(5) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_sim_callback(hObject, eventdata, sim)) ;
ui_cam = uicontrol('String', 'center', 'Position', [pbps(6) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_cam_callback(hObject, eventdata, sim, state)) ;
ui_quit = uicontrol('String', 'quit', 'Position', [pbps(7) pbrh pbw pbh], 'Callback', @(hObject, eventdata, handles) ui_quit_callback(hObject, eventdata, sim)) ;
ui_time = uicontrol('Style', 'text', 'String', 'Time:', 'Position', [pbps(8) 2.25*pbrh+1 pbw*4 pbh/2]) ;
ui_perror = uicontrol('Style', 'text', 'String', 'Position Error:', 'Position', [pbps(12) 2.25*pbrh+1 pbw*4 pbh/2]) ;
ui_yawerror = uicontrol('Style', 'text', 'String', 'Yaw Error:', 'Position', [pbps(16) 2.25*pbrh+1 pbw*4 pbh/2]) ;
ui_position = uicontrol('Style', 'text', 'String', 'Position:', 'Position', [pbps(8) pbrh-1 pbw*4 pbh/2]) ;
ui_velocity = uicontrol('Style', 'text', 'String', 'Velocity:', 'Position', [pbps(12) pbrh-1 pbw*4 pbh/2]) ;
ui_orientation = uicontrol('Style', 'text', 'String', 'Yaw:', 'Position', [pbps(16) pbrh-1 pbw*4 pbh/2]) ;


Draw3D ; % draw first frame
rotate3d on ;

pause(2) ; % give user some time to grab the window