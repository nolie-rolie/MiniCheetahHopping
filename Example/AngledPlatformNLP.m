clear
clc
close all
addpath(genpath([pwd() '/Models']))
addpath(genpath([pwd() '/spatial_v2_extended']))
addpath(genpath([pwd() '/Test']))
addpath(genpath([pwd() '/Utils']))
addpath(genpath([pwd() '/Visualization']))

%% robot specs

COM = [0,0,0.1]';
robot = getMiniCheetahParams();
paramsSRB = compute_SRBD_inertia(robot);
I = paramsSRB.RotInertia;
m = paramsSRB.mass;
M = diag([m m m]);
H = [inv(I),zeros(3,3);
     zeros(3,3),inv(M)];
h = 10e-3; % 10 ms
g = [0;0;0;0;0;9.81];

stance_time = 2e-1;
stance_length = stance_time/h;

nCtacts = 4;

% friction cone
mu = 0.5;
  
% FWPs
load("FWP_each_foot");
FWPFL = Polyhedron('V',V_FWPFL);
FWPFR = Polyhedron('V',V_FWPFR);
FWPHL = Polyhedron('V',V_FWPHL);
FWPHR = Polyhedron('V',V_FWPHR);
load('FFP.mat')
% for configuration constraint
maxHeightDiff = robot.hipLinkLength + robot.kneeLinkLength - COM(3);

%% problem setup

% import trajectory

load('AngledPlatformTrajectoryForNLP.mat')
state_traj_len = length(xd(1,:));
state_len = length(xd(:,1));
control_traj_len = length(ud(1,:));
control_len = length(ud(:,1));

ncd = [];
Fd = [];

for i = 1:control_traj_len
    for k = 1:nCtacts
        ncd(3*(k-1)+1:3*k,i) = skew((pfd(3*(k-1)+1:3*k,i) - xd(4:6,i)))*(ud(3*(k-1)+1:3*k,i));
    end
    
    Fd = [Fd, [sum(reshape(ncd(:,i),3,4),2);sum(reshape(ud(:,i),3,4),2)]];
end

%% NLP

% optimization variables
x = sdpvar(state_len,state_traj_len,'full');  %[eul;p;w;v]
u = sdpvar(control_len,control_traj_len,'full'); % GRFs [FL;FR;HL;HR]
nc = sdpvar(control_len,control_traj_len,'full'); % moment due to each contact

F = sdpvar(6,control_traj_len,'full'); % spatial wrench [n;f]

% smooth Fz - Bezier Curves
n  = 4;
nStance = 5;
alpha = sdpvar(6,n+1,nStance*2-2,'full');

% objective function
A = diag(ones(1,12));
B = diag(ones(1,12));

objective = 0;
for i = 1:state_traj_len
    objective = objective + (x(:,i)-xd(:,i))'*A*(x(:,i)-xd(:,i));
end
for i = 1:control_traj_len
    objective = objective + u(:,i)'*B*u(:,i);
end

% constraints
constr = [];

% start and end
constr = [constr, ...
          x(:,1)   == xd(:,1), ...
          x(:,end) == xd(:,end)];
      
% configuration
for i = 1:control_traj_len
    if isequal(ctacts(:,i),ones(4,1))
        constr = [constr, ...
                  -0.1 <= x(1,i) - xd(1,i) <= 0.1, ...
                  -0.1 <= x(2,i) - xd(2,i) <= 0.1, ...
                  -0.1 <= x(3,i) - xd(3,i) <= 0.1, ...
                  -0.1 <= x(4,i) - xd(4,i) <= 0.1, ...
                  -0.05 <= x(5,i) - xd(5,i) <= 0.05, ...
                  x(6,i) <= xd(6,i) + 0.2,...
                  x(6,i) >= xd(6,i) - 0.05];
    end
end

% spatial wrench in nominal FWP
% We use the FFP and enforce n = cross(r,f).
for i = 1:control_traj_len
    if ctacts(1,i) == 1
        constr = [constr, ...
                  FFPFR.A*u(1:3,i) <= FFPFR.b];
    else
        constr = [constr, ...
                  [u(1:3,i);nc(1:3,i)] == zeros(6,1)];
    end
    if ctacts(2,i) == 1
        constr = [constr, ...
                  FFPFL.A*u(4:6,i) <= FFPFL.b];
    else
        constr = [constr, ...
                  [u(4:6,i);nc(4:6,i)] == zeros(6,1)];
    end
    if ctacts(3,i) == 1
        constr = [constr, ...
                  FFPHR.A*u(7:9,i) <= FFPHR.b];
    else
        constr = [constr, ...
                  [u(7:9,i);nc(7:9,i)] == zeros(6,1)];
    end
    if ctacts(4,i) == 1
        constr = [constr, ...
                  FFPHL.A*u(10:12,i) <= FFPHL.b];
    else
        constr = [constr, ...
                  [u(10:12,i);nc(10:12,i)] == zeros(6,1)];
    end
end

% dynamics
for i = 1:control_traj_len
    % rotation for world inertia matrix
	Rz = [ cos(x(3,i)),  -sin(x(3,i)), 0;
           sin(x(3,i)),   cos(x(3,i)), 0;
           0,             0,           1];
    Ry = [ cos(x(2,i)), 0,  sin(x(2,i));
           0,           1,  0;
          -sin(x(2,i)), 0,  cos(x(2,i))];
    Rx = [1, 0,            0;
          0, cos(x(1,i)), -sin(x(1,i));
          0, sin(x(1,i)),  cos(x(1,i))];
      
    for j = 1:nCtacts
        constr = [constr, ...
                  nc(3*(j-1)+1:3*j,i) == skew((Rz*Ry*Rx)'*(pfd(3*(j-1)+1:3*j,i) - x(4:6,i)))*(u(3*(j-1)+1:3*j,i))];
    end
    
    % discrete time dynamics
    constr = [constr, ...
              F(:,i) == [nc(1:3,i)+nc(4:6,i)+nc(7:9,i)+nc(10:12,i); ...
                         u(1:3,i)+u(4:6,i)+u(7:9,i)+u(10:12,i)], ...
                         x(7:9,i+1) == x(7:9,i) + h * (Rz*Ry*Rx*H(1:3,1:3)*F(1:3,i)), ...
                         x(10:12,i+1) == x(10:12,i) + h * (Rz*Ry*Rx*H(4:6,4:6)*F(4:6,i) - g(4:6))
              ];

    constr = [constr, ...
              x(1:6,i+1) == x(1:6,i) + h*x(7:12,i)];

end

% bezier curves
for i = 1:nStance*2-2
    constr = [constr, ... 
              alpha(1:5,1,i) == zeros(5,1), ...
              alpha(6,1,i) == m*g(6), ...
              alpha(1:5,end,i) == zeros(5,1)
              ];
      
    constr = [constr, ...
              alpha(4,:,i) <= 4*mu/sqrt(2)*150, ...
              alpha(4,:,i) <= 4*mu/sqrt(2)*150, ...
              alpha(5,:,i) <= 4*mu/sqrt(2)*150, ...
              alpha(5,:,i) <= 4*mu/sqrt(2)*150, ...
              0 <= alpha(6,:,i) <= 4*150
              ];
end

constr = [constr, ...
           F(4,:) <= 4*mu/sqrt(2)*150, ...
          -F(4,:) <= 4*mu/sqrt(2)*150, ...
           F(5,:) <= 4*mu/sqrt(2)*150, ...
          -F(5,:) <= 4*mu/sqrt(2)*150, ...
           0 <= F(6,:) <= 4*150
           ];

stance = 1;
stance_start = ones(nStance,1);
N = zeros(nStance*2-2,1);
indStance = 1;
B = {};
for i = 1:control_traj_len
    if stance == 1
        if ctacts(1,i) == 1 && i ~= control_traj_len
            N(indStance) = N(indStance) + 1;
        else
            if i == control_traj_len
                N(indStance) = N(indStance) + 1;
            end
            stance = 0;
            
            for j = 1:N(indStance)
                for k = 1:n+1
                    B{indStance}(k,j) = nchoosek(n,(k-1))*(1-(j-1)/(N(indStance)-1))^(n-(k-1))*((j-1)/(N(indStance)-1))^(k-1);
                end
               
                constr = [constr, ...
                              F(:,(j-1)+stance_start(indStance)) == alpha(:,:,indStance)*B{indStance}(:,j)
                              ];
                      
            end
            
        end
    else
        if ctacts(1,i+1) == 1 
            stance = 1;
            indStance = indStance + 1;
            stance_start(indStance) = i+1;
            
        end
    end
end

% supply initial solution
assign(x,xd)
assign(u,ud)
assign(nc,ncd)
assign(F,Fd)

options = sdpsettings('verbose',2,'solver','ipopt','debug',1,'showprogress',10, ...
                        'usex0',1,'ipopt.max_cpu_time',60);
sol = optimize(constr,objective,options);

%% plot solution

t = 0:h:h*(state_traj_len-1);

figure(1);
subplot(2,1,1)
plot(t,value(x(4,:)),'r')
hold on
plot(t,value(x(5,:)),'g')
hold on
plot(t,value(x(6,:)),'b')
plot(t,xd(4,:),'r--')
hold on
plot(t,xd(5,:),'g--')
hold on
plot(t,xd(6,:),'b--')
title('COM Position')
ylabel('Position (m)')
xlabel('Time (sec)')
legend('x_{NLP}','y_{NLP}','z_{NLP}','x_{MIP}','y_{MIP}','z_{MIP}')

subplot(2,1,2)
plot(t,value(x(1,:)),'r')
hold on
plot(t,value(x(2,:)),'g')
hold on
plot(t,value(x(3,:)),'b')
plot(t,xd(1,:),'r--')
hold on
plot(t,xd(2,:),'g--')
hold on
plot(t,xd(3,:),'b--')
title('COM Orientation')
ylabel('Angle (rad)')
xlabel('Time (sec)')
legend('r_{NLP}','p_{NLP}','y_{NLP}','r_{MIP}','p_{MIP}','y_{MIP}')

%% velocity

figure(11);
subplot(2,1,1)
plot(t,value(x(7,:)),'r')
hold on
plot(t,value(x(8,:)),'g')
hold on
plot(t,value(x(9,:)),'b')
plot(t,xd(7,:),'r--')
hold on
plot(t,xd(8,:),'g--')
hold on
plot(t,xd(9,:),'b--')
title('COM Rotational Velocity')
ylabel('w (rad/s)')
xlabel('Time (sec)')
legend('rdot_{NLP}','pdot_{NLP}','ydot_{NLP}','rdot_{MIP}','pdot_{MIP}','ydot_{MIP}')

subplot(2,1,2)
plot(t,value(x(10,:)),'r')
hold on
plot(t,value(x(11,:)),'g')
hold on
plot(t,value(x(12,:)),'b')
plot(t,xd(10,:),'r--')
hold on
plot(t,xd(11,:),'g--')
hold on
plot(t,xd(12,:),'b--')
title('COM Velocity')
ylabel('Velocity (m/s)')
xlabel('Time (sec)')
legend('xdot_{NLP}','ydot_{NLP}','zdot_{NLP}','xdot_{MIP}','ydot_{MIP}','xdot_{MIP}')

figure(2)
subplot(2,2,1)
plot(t(1:end-1),value(u(1,:)),'r')
hold on
plot(t(1:end-1),value(u(2,:)),'g')
hold on
plot(t(1:end-1),value(u(3,:)),'b')
plot(t(1:end-1),ud(4,:),'r--')
hold on
plot(t(1:end-1),ud(5,:),'g--')
hold on
plot(t(1:end-1),ud(6,:),'b--')
title('GRFs for Front Left Foot')
xlabel('Time (sec)')
ylabel('Force (Newtons)')
legend('F_{x-NLP}','F_{y-NLP}','F_{z-NLP}','F_{x-MIP}','F_{y-MIP}','F_{z-MIP}')

subplot(2,2,2)
plot(t(1:end-1),value(u(4,:)),'r')
hold on
plot(t(1:end-1),value(u(5,:)),'g')
hold on
plot(t(1:end-1),value(u(6,:)),'b')
plot(t(1:end-1),ud(4,:),'r--')
hold on
plot(t(1:end-1),ud(5,:),'g--')
hold on
plot(t(1:end-1),ud(6,:),'b--')
title('GRFs for Front Right Foot')
xlabel('Time (sec)')
ylabel('Force (Newtons)')
legend('F_{x-NLP}','F_{y-NLP}','F_{z-NLP}','F_{x-MIP}','F_{y-MIP}','F_{z-MIP}')

subplot(2,2,3)
plot(t(1:end-1),value(u(7,:)),'r')
hold on
plot(t(1:end-1),value(u(8,:)),'g')
hold on
plot(t(1:end-1),value(u(9,:)),'b')
plot(t(1:end-1),ud(7,:),'r--')
hold on
plot(t(1:end-1),ud(8,:),'g--')
hold on
plot(t(1:end-1),ud(9,:),'b--')
title('GRFs for Hind Left Foot')
xlabel('Time (sec)')
ylabel('Force (Newtons)')
legend('F_{x-NLP}','F_{y-NLP}','F_{z-NLP}','F_{x-MIP}','F_{y-MIP}','F_{z-MIP}')

subplot(2,2,4)
plot(t(1:end-1),value(u(10,:)),'r')
hold on
plot(t(1:end-1),value(u(11,:)),'g')
hold on
plot(t(1:end-1),value(u(12,:)),'b')
plot(t(1:end-1),ud(10,:),'r--')
hold on
plot(t(1:end-1),ud(11,:),'g--')
hold on
plot(t(1:end-1),ud(12,:),'b--')
title('GRFs for Hind Right Foot')
xlabel('Time (sec)')
ylabel('Force (Newtons)')
legend('F_{x-NLP}','F_{y-NLP}','F_{z-NLP}','F_{x-MIP}','F_{y-MIP}','F_{z-MIP}')

%% plot spatial wrench

F = value(F);
figure(3)
subplot(2,1,1)
plot(t(1:end-1),F(1,:),'r--')
hold on
plot(t(1:end-1),F(2,:),'g--')
hold on
plot(t(1:end-1),F(3,:),'b--')
legend('nx','ny','nz')
subplot(2,1,2)
plot(t(1:end-1),F(4,:),'r')
hold on
plot(t(1:end-1),F(5,:),'g')
hold on
plot(t(1:end-1),F(6,:),'b')
legend('Fx','Fy','Fz');

%% visualization
   
% trajectory
eul = value(x(1:3, :));
pos = value(x(4:6, :));
qdummy = pfd;
GRF = value(u);

% fill data
graphic_option.show_footloc = true;
graphic_option.show_floor = true;
graphic_option.show_platforms = true;
graphic_option.show_obstacles = false;
graphic_option.show_angled_platforms = true;
graphic_option.setCameraPos = true;
graphic_option.show_GRF = true;
graphic_option.hide_leg = false;
graphics_data.platforms = platforms;

Ni = 3;
graphics_data.eul = eul;
graphics_data.pos = pos;
graphics_data.qdummy = qdummy;
graphics_data.F = GRF;
graphics_data.ctacts = ctacts;
graphics_data.time = 1:Ni:size(GRF, 2);
graphics_data.platforms = platforms;
graphics_data.angled_platforms = angled_platforms;
graphics_data.CameraPos = [0,-5,2];
graphics_data.CameraPos = [-10,0,2];

% call visualize function
out = visualizeMCTrajectory(graphics_data, graphic_option);

