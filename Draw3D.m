verts = bsxfun(@plus,(state.R * vertsOrig'),state.p)' ; % translate and rotate original boat to current state

% Plot stl boat graphic
object.vertices = verts;
object.faces = faces;
boatGraphic = patch(object,'FaceColor',       [0.2 0.2 0.2], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15) ;
material('dull') ; % body material

% Update displayed text
timestring = strcat({'Time:  '}, num2str(round(state.t.*100)./100)) ;
set(ui_time, 'String', timestring) ;
perrorstring = strcat({'Position Error:  '}, num2str(round([robot.pDes(1)-state.p(1), robot.pDes(2)-state.p(2)].*100)./100)) ;
set(ui_perror, 'String', perrorstring) ;
yawerrorstring = strcat({'Yaw Error:  '}, num2str(round((robot.yDes(1)-state.th(3)).*(180/pi).*100)./100)) ;
set(ui_yawerror, 'String', yawerrorstring) ;
positionstring = strcat({'Position:  '}, num2str(round([state.p(1), state.p(2), state.p(3)].*100)./100)) ;
set(ui_position, 'String', positionstring) ;
velocitystring = strcat({'Velocity:  '}, num2str(round([state.v(1), state.v(2), state.v(3)].*100)./100)) ;
set(ui_velocity, 'String', velocitystring) ;
orientationstring = strcat({'Yaw:  '}, num2str(round([state.th(3)].*100.*(180/pi))./100), {'          '}, 'Rate:', num2str(round([state.w(3)].*(180/pi).*100)./100)) ;
set(ui_orientation, 'String', orientationstring) ;

% Draw water
waterGraphic = patch([sim.windowSize(1),sim.windowSize(2),sim.windowSize(2),sim.windowSize(1)],[sim.windowSize(3),sim.windowSize(3),sim.windowSize(4),sim.windowSize(4)],'blue','FaceAlpha',0.25) ;

% Draw desired waypoint
wpel = 0.25 ;
waypointRec = [robot.pDes(1)-wpel, robot.pDes(1)+wpel, robot.pDes(2)-wpel, robot.pDes(2)+wpel] ;
waypointGraphic_p = patch([waypointRec(1),waypointRec(2),waypointRec(2),waypointRec(1)],[waypointRec(3),waypointRec(3),waypointRec(4),waypointRec(4)],'red','FaceAlpha',0.75) ;
hDes = [cos(robot.yDes), sin(robot.yDes)] ;
wph = 1 ;
waypointGraphic_y = line([robot.pDes(1), robot.pDes(1)+hDes(1)], [robot.pDes(2), robot.pDes(2)+hDes(2)], [wph, wph], 'Color', 'red') ;
waypointGraphic_y_ep = scatter3(robot.pDes(1), robot.pDes(2), wph, 'filled', 'MarkerFaceColor', 'red') ;

% Draw thrusts
%%% TBI