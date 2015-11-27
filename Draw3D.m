verts = bsxfun(@plus,(state.R * vertsOrig'),state.p)' ; % translate and rotate original boat to current state
% Plot stl boat graphic
object.vertices = verts;
object.faces = faces;
BoatGraphic = patch(object,'FaceColor',       [0.2 0.2 0.2], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15) ;
material('dull') ; % body material


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DEBUG LINES
% hold on
% 
% L = state.R*boat.I*state.R' * state.w ;
% Ln = L./norm(L) ;
% wMag = norm(state.w) ;
%     if(wMag >= 0.00001)
%         u = state.w / wMag ;
%     else
%         u = [0;0;0] ;
%     end
%     
% s = 2 ;
% try
%     delete(Line1) ;
%     delete(Line2) ;
% catch
% end
% Line1 = line([0,s*u(1)],[0,s*u(2)],[0,s*u(3)],'Color','r','LineWidth',3) ;
% Line2 = line([0,s*Ln(1)],[0,s*Ln(2)],[0,s*Ln(3)],'Color','b','LineWidth',3) ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%