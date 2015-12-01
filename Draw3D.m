verts = bsxfun(@plus,(state.R * vertsOrig'),state.p)' ; % translate and rotate original boat to current state
% Plot stl boat graphic
object.vertices = verts;
object.faces = faces;
BoatGraphic = patch(object,'FaceColor',       [0.2 0.2 0.2], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15) ;
material('dull') ; % body material

% Update displayed time
timestring = strcat('Time: ', num2str(state.t)) ;
set(ui_time, 'String', timestring) ;

% Draw thrusts
%%% TBI