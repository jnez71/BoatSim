% Sum forces and moments on boat
F = Fg + Fb + Fd + Fw + Fv + env.Fz + Ft ;
M = Mg + Mb + Md + Mw+ Mv + state.R'*env.Mz + Mt ;
% F = [0;0;0] ; % free body
% M = [0;0;0] ; % free body

% translational
state.p = state.p + state.v*sim.dt ;
accel = F / boat.m ;
state.v = state.v + accel*sim.dt ;

% rotational
L = state.R*boat.I*state.R' * state.w ;
wMag = norm(state.w) ;
wb = state.R'*state.w ;
if(wMag >= 0.00001)
u = wb / wMag ;
uCross = [0,-u(3),u(2);u(3),0,-u(1);-u(2),u(1),0] ;
uTens = [u(1)^2,u(1)*u(2),u(1)*u(3);u(1)*u(2),u(2)^2,u(2)*u(3);u(1)*u(3),u(2)*u(3),u(3)^2] ;
dth = wMag * sim.dt ;
state.R = state.R * (cos(dth)*eye(3)+sin(dth)*uCross+(1-cos(dth))*uTens) ;
state.th = [atan2(state.R(3,2),state.R(3,3)) ; atan2(-state.R(3,1),sqrt(state.R(3,2)^2+state.R(3,3)^2)) ; atan2(state.R(2,1),state.R(1,1))] ;
end
L = L + state.R*M*sim.dt ;
state.w = (state.R*boat.I*state.R') \ L ;

% time increment
state.t = state.t + sim.dt ;