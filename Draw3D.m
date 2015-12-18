% Translate and rotate original boat to current state
verts = bsxfun(@plus,(state.R * vertsOrig'),state.p)' ;

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
errorstring = strcat({'Error:  '}, num2str(round([robot.pDes(1)-state.p(1), robot.pDes(2)-state.p(2), (robot.yDes(1)-state.th(3)).*(180/pi)].*100)./100)) ;
set(ui_error, 'String', errorstring) ;
if(strcmp(boat.type, 'azi'))
    thruststring = strcat({'Command:  '}, num2str(round([state.thrusters(1:2),state.thrusters(3:4).*(180/pi)].*100)./100)) ;
else
    thruststring = strcat({'Command:  '}, num2str(round(state.thrusters.*100)./100)) ;
end
set(ui_thrust, 'String', thruststring) ;
adaptstring = strcat({'Adaptive:  '}, num2str(round([robot.adaptDrag(1), robot.adaptDrag(2), robot.adaptDrag(3), robot.adaptDrag(4), robot.adaptDrag(5)].*100)./100)) ;
set(ui_adapt, 'String', adaptstring) ;
positionstring = strcat({'Position:  '}, num2str(round([state.p(1), state.p(2), state.p(3)].*100)./100)) ;
set(ui_position, 'String', positionstring) ;
velocitystring = strcat({'Velocity:  '}, num2str(round([state.v(1), state.v(2), state.v(3)].*100)./100)) ;
set(ui_velocity, 'String', velocitystring) ;
orientationstring = strcat({'Yaw:  '}, num2str(round([state.th(3)].*100.*(180/pi))./100), {'          '}, 'Rate:', num2str(round([state.w(3)].*(180/pi).*100)./100)) ;
set(ui_orientation, 'String', orientationstring) ;
integstring = strcat({'Integral:  '}, num2str(round([robot.adaptConst(1), robot.adaptConst(2), robot.adaptConst(3)].*100)./100)) ;
set(ui_integ, 'String', integstring) ;

% Draw water
if env.active
    waterGraphic = patch([sim.windowSize(1),sim.windowSize(2),sim.windowSize(2),sim.windowSize(1)],[sim.windowSize(3),sim.windowSize(3),sim.windowSize(4),sim.windowSize(4)],'blue','FaceAlpha',0.25) ;
end
    
% Draw desired waypoint
wpel = 0.25 ;
waypointRec = [robot.pDes(1)-wpel, robot.pDes(1)+wpel, robot.pDes(2)-wpel, robot.pDes(2)+wpel] ;
waypointGraphic_p = patch([waypointRec(1),waypointRec(2),waypointRec(2),waypointRec(1)],[waypointRec(3),waypointRec(3),waypointRec(4),waypointRec(4)],'green','FaceAlpha',0.75) ;
hDes = [cos(robot.yDes), sin(robot.yDes)] ;
wph = 1 ;
waypointGraphic_y = line([robot.pDes(1), robot.pDes(1)+hDes(1)], [robot.pDes(2), robot.pDes(2)+hDes(2)], [wph, wph], 'Color', 'green') ;
waypointGraphic_y_ep = scatter3(robot.pDes(1), robot.pDes(2), wph, 30, 'filled', 'MarkerFaceColor', 'green', 'MarkerEdgeColor','k') ;

% Draw thrusts
if sim.drawThrusts
    boat.type = boat.type_next ;
    Theight = 0.5 ;
    Tscale = 0.5 * 1/boat.maxT ;
    
    if(strcmp(boat.type, 'azi'))
        
        Tbl = state.thrusters(1) ;
        Tbr = state.thrusters(2) ;
        phibl = state.thrusters(3) ;
        phibr = state.thrusters(4) ;
        
        pTbl = state.p + state.R*boat.Lbl ;
        pTbr = state.p + state.R*boat.Lbr ;
        
        hTbl = Tscale*Tbl*state.R*[cos(phibl), sin(phibl), 0]' ;
        hTbr = Tscale*Tbr*state.R*[cos(phibr), sin(phibr), 0]' ;

        thruster_bl = line([pTbl(1), pTbl(1)+hTbl(1)], [pTbl(2), pTbl(2)+hTbl(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
        thruster_br = line([pTbr(1), pTbr(1)+hTbr(1)], [pTbr(2), pTbr(2)+hTbr(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
        
        thruster_bl_ep = scatter3(pTbl(1), pTbl(2), Theight, 20, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
        thruster_br_ep = scatter3(pTbr(1), pTbr(2), Theight, 20, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
        
    elseif(strcmp(boat.type, 'fixed'))
        
        Tbl = state.thrusters(1) ;
        Tbr = state.thrusters(2) ;
        Tfl = state.thrusters(3) ;
        Tfr = state.thrusters(4) ;
        
        pTbl = state.p + state.R*boat.Lbl ;
        pTbr = state.p + state.R*boat.Lbr ;
        pTfl = state.p + state.R*boat.Lfl ;
        pTfr = state.p + state.R*boat.Lfr ;
        
        hTbl = Tscale*Tbl*state.R*boat.dbl ;
        hTbr = Tscale*Tbr*state.R*boat.dbr ;
        hTfl = Tscale*Tfl*state.R*boat.dfl ;
        hTfr = Tscale*Tfr*state.R*boat.dfr ;
        
        thruster_bl = line([pTbl(1), pTbl(1)+hTbl(1)], [pTbl(2), pTbl(2)+hTbl(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
        thruster_br = line([pTbr(1), pTbr(1)+hTbr(1)], [pTbr(2), pTbr(2)+hTbr(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
        thruster_fl = line([pTfl(1), pTfl(1)+hTfl(1)], [pTfl(2), pTfl(2)+hTfl(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
        thruster_fr = line([pTfr(1), pTfr(1)+hTfr(1)], [pTfr(2), pTfr(2)+hTfr(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
    
        thruster_bl_ep = scatter3(pTbl(1), pTbl(2), Theight, 20, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
        thruster_br_ep = scatter3(pTbr(1), pTbr(2), Theight, 20, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
        thruster_fl_ep = scatter3(pTfl(1), pTfl(2), Theight, 20, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
        thruster_fr_ep = scatter3(pTfr(1), pTfr(2), Theight, 20, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
    
    elseif(strcmp(boat.type, 'direct'))
        
        forcescale = 1/80 ;
        torquescale = 1/60 ;
        
        forceGraphic = line([state.p(1), state.p(1) + forcescale*state.thrusters(1)], [state.p(2), state.p(2) + forcescale*state.thrusters(2)], [Theight, Theight], 'Color', 'red', 'LineWidth', 1.25) ;
        forceGraphic_ep = scatter3(state.p(1), state.p(2), Theight, 30, 'MarkerFaceColor', 'red', 'MarkerEdgeColor','k') ;
        torqueGraphic = line([state.p(1), state.p(1)], [state.p(2), state.p(2)], [Theight, Theight + torquescale*state.thrusters(3)], 'Color', 'm', 'LineWidth', 1.25) ;        
        
    end
end
