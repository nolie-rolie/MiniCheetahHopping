function p = leg_kinematics(q,leg)
% Leg kinematics in local frame, where the orientation of local frame is
% the same as the body frame, and its origin is at the center of abad
% p-> foot position in local frame

l0 = 0.062; % abad link length
l1 = 0.209; % hip link length
l2 = 0.195; % knee link length

s = @(x) sin(x);
c = @(x) cos(x);
Rx = @(x) [1,0,0;0 c(x) -s(x);0 s(x) c(x)];
Ry = @(x) [c(x),0,s(x);0 1 0;-s(x),0,c(x)];
sign = (-1)^leg;

T1 = rotm2tform(Rx(q(1))) * trvec2tform(sign*[0 l0 0]);
T2 = rotm2tform(Ry(-q(2))) * trvec2tform([0 0 -l1]);
T3 = rotm2tform(Ry(-q(3)));

p = hom2cart(T1*T2*T3*cart2hom([0 0 -l2])');
end