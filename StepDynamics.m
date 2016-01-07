% Sum forces and moments on boat
if env.active
    F = Fg + Fb + Fd + Fw + Fv + env.Fz ;
    M = Mg + Mb + Md + Mw + Mv + env.Mz ;
else
    F = [0;0;0] ;
    M = [0;0;0] ;
end

if robot.active && (state.p(3) < boat.height)
    F = F + Ft ;
    M = M + Mt ;
end

% Put output of interest into state for recording
state.output = robot.adaptDrag ;

% Translational increment
state.p = state.p + state.v*sim.dt ;
accel = F / boat.m ;
state.v = state.v + accel*sim.dt ;

% Rotational increment
L = state.R*boat.I*state.R' * state.w ; % angular momentum
wMag = norm(state.w) ;
wb = state.R'*state.w ;
if(wMag >= 0.00001)
u = wb / wMag ;
uCross = [0,-u(3),u(2);u(3),0,-u(1);-u(2),u(1),0] ;
uTens = [u(1)^2,u(1)*u(2),u(1)*u(3);u(1)*u(2),u(2)^2,u(2)*u(3);u(1)*u(3),u(2)*u(3),u(3)^2] ;
dth = wMag * sim.dt ;
state.R = state.R * (cos(dth)*eye(3)+sin(dth)*uCross+(1-cos(dth))*uTens) ;
detR = det(state.R) ;
if detR >= 1.0001 || detR <= 0.9999 % avoid round-off erroring away from SO3
    fprintf('\nRenormalizing orientation state!\n') ;
    [state.R, notactuallyR] = qr(state.R) ;
    if det(state.R) < 0
        state.R(:,3) = -state.R(:,3) ;
    end
end 
state.th = [atan2(state.R(3,2),state.R(3,3)) ; atan2(-state.R(3,1),sqrt(state.R(3,2)^2+state.R(3,3)^2)) ; atan2(state.R(2,1),state.R(1,1))] ;
state.heading = state.R(:,1) ;
end
L = L + state.R*M*sim.dt ;
state.w = (state.R*boat.I*state.R') \ L ;

% Time increment
state.t = state.t + sim.dt ;