function Jv = compute_foot_jacobian(robot, pos, eul, qleg, leg)
% robot -> robot tree built with spatial_v2
% pos -> position of CoM
% eul -> euler angle relative to moving frame
% qleg -> joint angle associated with leg
% leg -> leg ID [1,2,3,4]->[FR, FL , HR, HL]
% Jv -> Foot Jacobian for linear velicity in worldframe
% All components that relate to other legs should be zero in Jv

kneeLinkLength = 0.195;
Xtree_knee_to_foot = plux(eye(3), [0, 0, -kneeLinkLength]');
S = zeros(12,3);
S(3*(leg-1)+1:3*leg, :) = eye(3);
qJ = S*qleg;
q = [pos; eul; qJ];
% Get the jacobian expressed in foot frame
J_local = BodyJacobian(robot, q, LINKID.knee(leg), Xtree_knee_to_foot);

[~, ~, info] = HandC(robot, q, zeros(size(q)));
Xup = info.Xup;
X = eye(6);

j = LINKID.knee(leg);

while j > 0
    X = X * Xup{j};
    j = robot.parent(j);
end


[R, ~] = plux(X);

J_world = blkdiag(R', R') * J_local;
Jv = J_world(4:end, :);
end