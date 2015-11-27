[vertsOrig, faces, normals, stlName] = stlread('PropaGator_Body.STL') ; % load boat model
vertsOrig = (0.0254 * eye(3) * vertsOrig')' ; % scale boat model to meters

figure ; % open figure window
grid on ; % draw grid lines so motion is more apparent

axis(sim.windowSize) ; % set axis limits
axis('image') ; % set uniform axis scaling for proper image display
view(sim.view0) ; % set initial viewing angle
camlight('headlight') ; % lighting

hold on ; % lock this figure as the active figure

% Other things to draw in:
water = patch([sim.windowSize(1),sim.windowSize(2),sim.windowSize(2),sim.windowSize(1)],[sim.windowSize(1),sim.windowSize(1),sim.windowSize(2),sim.windowSize(2)],'blue','FaceAlpha',0.25) ;

Draw3D ; % draw first frame
pause(2) ; % give user some time to grab the window