function traj = saveCOMTrajToFile(x,v)
% x = [pos]; v = [vel] 
% input in order [FL;FR;HL;HR]

%% get data to desired format 
% interpolate
time = 0:0.01:0.01*(length(x(1,:))-1);
t = 0:0.002:0.01*(length(x(1,:))-1);
x_des = interp1(time,x',t)';
v_des = interp1(time,v',t)';
        
%% save in array
traj = [x_des;v_des];
