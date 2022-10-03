function traj = saveTrajToFile(q,u)
% x = [pos;eul], q = [joint angles], u = [GRFS] 
% input in order [FL;FR;HL;HR]

%% get data to desired format 
% interpolate
time = 0:0.01:0.01*(length(q(1,:))-1);
t = 0:0.002:0.01*(length(q(1,:))-1);
q_des = interp1(time,q',t)';

for i = 1:length(t)
    ind = 1+floor(i/(length(t)/length(time)));
    ind = min(ind,length(u(1,:)));
    f_ff(:,i) = u(:,ind);
end

f_ff = interp1(time,u',t)';
        
% get approximate joint velocities
qd_des = diff(q_des,1,2);
qd_des = [qd_des,zeros(12,1)];

%% save in array
traj = [f_ff;q_des;qd_des];
