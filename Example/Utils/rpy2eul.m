function eul = rpy2eul(rpy)
% takes fixed-frame roll-pitch-yaw angles to moving-frame ZYX Euler angles
eul = rpy([3,2,1],:);
end