function p = forward_kinematics(pos, eul, qleg, leg)
% compute the foot position for leg
% abadLoc{1} = [0.19, -0.049, 0]';     % FR
% abadLoc{2} = [0.19, 0.049, 0]';      % FL
% abadLoc{3} = [-0.19, -0.049, 0]';    % RR
% abadLoc{4} = [-0.19, 0.049, 0]';     % RL

abadLoc = [(-1)^(floor(leg/3))*0.19, (-1)^(leg)*0.049, 0]';
R = eul2Rot(eul);
p = R*(abadLoc + leg_kinematics(qleg, leg)) + pos;
end