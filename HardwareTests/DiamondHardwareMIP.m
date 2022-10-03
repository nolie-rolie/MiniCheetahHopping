clc
clear
close all


%% robot specifications
rNominal = [ 0.19, -0.1110, 0;
             0.19,  0.1110, 0;
            -0.19, -0.1110, 0;
            -0.19,  0.1110, 0]; % [fr,fl,br,bl]
        
nFeet = length(rNominal(:,1));

COM = [0 0 0.28]'; % COM position relative to "contact point"
COM(3) = 0.18;
r2COM = zeros(4,length(COM));
for i = 1:nFeet
    r2COM(i,:) = rNominal(i,:) - COM';
end

fzMAX = 150; % Max force in z-direction for each contact (Newtons) ... I'm making this up :0
load('FWP_Mini_Cheetah_mu_07.mat');
FWP = Polyhedron('V',V_FWP);
A_FWP = FWP.A;
b_FWP = FWP.b;

robot = getMiniCheetahParams();
paramsSRB = compute_SRBD_inertia(robot);
I = paramsSRB.RotInertia;

h1 = 2e-1;
h = 2e-1; % timestep for discrete time system dynamics ... I'm making this up :0
h = 6e-1;

m = 9; % mass in kg 
M = diag([m m m]);

radius = norm(r2COM(1,:));
radius_platform = norm(rNominal(1,:));
mu = 0.5; % friction coefficient

%% lay out environment

% flat platforms
length = 0.3;
gap = 0.1;
platform1_xLB = -length;
platform1_xUB = length;
platform1_yLB = -length;
platform1_yUB = length;
platform1_z= 0;

platform2_xLB = platform1_xUB + gap;
platform2_xUB = platform2_xLB+length*2;
platform2_yLB = -length*2 - gap/2;
platform2_yUB = -gap/2;
platform2_z = 0;

platform3_xLB = platform1_xUB + gap;
platform3_xUB = platform3_xLB+length*2;
platform3_yLB = gap/2;
platform3_yUB = length*2 + gap/2;
platform3_z = 0;

platform4_xLB = platform3_xUB + gap;
platform4_xUB = platform4_xLB + length*2;
platform4_yLB = platform1_yLB;
platform4_yUB = platform1_yUB;
platform4_z = 0;

platforms = [platform1_xLB platform1_xUB platform1_yLB platform1_yUB platform1_z;
             platform2_xLB platform2_xUB platform2_yLB platform2_yUB platform2_z;
             platform3_xLB platform3_xUB platform3_yLB platform3_yUB platform3_z;
             platform4_xLB platform4_xUB platform4_yLB platform4_yUB platform4_z;];

nPlatforms = size(platforms,1);

% points
stPoint = [0, 0, platforms(1,5), 0, 0, 0];

%% plot environment

figure(1)

% points
plot3([stPoint(1)],[stPoint(2)], [stPoint(3)],'k*')
hold on

% platforms

for i = 1:nPlatforms
    p1 = [platforms(i,1) platforms(i,3) platforms(i,5)];
    p2 = [platforms(i,2) platforms(i,3) platforms(i,5)];
    p3 = [platforms(i,2) platforms(i,4) platforms(i,5)];
    p4 = [platforms(i,1) platforms(i,4) platforms(i,5)]; 

    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];

    fill3(x, y, z, [0.7,0.7,0.7]);
    xlabel('x'); ylabel('y'); zlabel('z'); 
    hold on
end


%% adjust platforms width/length to ensure all feet land on platforms

platforms_true = platforms;
for i = 1:nPlatforms
    platforms(i,1) = platforms_true(i,1) + radius_platform;
    platforms(i,2) = platforms_true(i,2) - radius_platform;
    platforms(i,3) = platforms_true(i,3) + radius_platform;
    platforms(i,4) = platforms_true(i,4) - radius_platform;
end

%% formulate and solve MIP problem
nPoints = 10; % increase to decrease prob of hitting obstacle
nHops = 2; 
nBoundaries = 6;
gravity = [0 0 -9.8 0 0 0]';
bigM = 1e4;
epsilon = 1e-3;
safety = 0;%10e-2;

% state variables
qCOM = sdpvar(3,nPoints,nHops,'full');  % COM position

qTD = sdpvar(6,nHops+1,'full');    % Contact points
zTD = binvar(nPlatforms,nHops,'full'); % binaries to restrict footstep position

qdotTO = sdpvar(6,nHops+1,'full'); % [take off velocity]
Tair = sdpvar(1,nHops,'full');   % time spent in air for each hop 
T2 = sdpvar(1,nHops,'full');
zT2 = binvar(3,nHops,'full'); % 3 linear pieces

fnet = sdpvar(3,nHops+1,'full');
n = sdpvar(3,nHops+1,'full'); % torque

C = sdpvar(1,nHops+1,'full'); % piecewise cos
zC = binvar(9,nHops+1,'full'); % binaries for piecewise cos
S = sdpvar(1,nHops+1,'full'); % piecewise sin
zS = binvar(9,nHops+1,'full'); % binaries piecewise sin
CYawBounds = [-2*pi,    -15*pi/8; ...
              -15*pi/8, -9*pi/8; ...
              -9*pi/8,  -7*pi/8; ...
              -7*pi/8,  -pi/8; ...
              -pi/8,     pi/8; ...
               pi/8,     7*pi/8; ...
              7*pi/8,    9*pi/8; ...
              9*pi/8,    15*pi/8; ...
              15*pi/8,   2*pi]; 
SYawBounds = [-2*pi,    -13*pi/8; ...
              -13*pi/8, -11*pi/8; ...
              -11*pi/8, -5*pi/8; ...
              -5*pi/8,  -3*pi/8; ...
              -3*pi/8,   3*pi/8; ...
               3*pi/8,    5*pi/8; ...
               5*pi/8,    11*pi/8; ...
              11*pi/8,   13*pi/8; ...
              13*pi/8,   2*pi];
L = 1;
U = 2;
TBounds = [0,   1/3; ...
           1/3, 2/3; ...
           2/3, 1];

% force longer stance time
zST = binvar(1,nHops,'full');

constr = [];
% start and end constraints
constr = [qTD(:,1) == stPoint', ... % Start point
          qTD(1,nHops+1) >= platforms(end,1) + safety, ... % Constrain last hop to final platform
          qTD(1,nHops+1) <= platforms(end,2) - safety, ...
          qTD(2,nHops+1) >= platforms(end,3) + safety, ...
          qTD(2,nHops+1) <= platforms(end,4) - safety, ...
          qTD(3,nHops+1) == platforms(end,5), ...
          qTD(4,nHops+1) == 0, ...
          qTD(5,nHops+1) == 0, ...
          qdotTO(:,nHops+1) == [0;0;0;0;0;0]
          qCOM(:,end,nHops) == qTD(1:3,end) + COM];
      
% piecewise linear approximation of T^2
constr = [constr, ...
          0 <= Tair(:) <= 1, ...
          0 <= T2(:) <= 1];
for i = 1:nHops
    T2_1 = implies(zT2(1,i), [TBounds(1,L) <= Tair(i) <= TBounds(1,U), T2(i) == 1/3*Tair(i)]);
    T2_2 = implies(zT2(2,i), [TBounds(2,L) <= Tair(i) <= TBounds(2,U), T2(i) == Tair(i) - 2/9]);
    T2_3 = implies(zT2(3,i), [TBounds(3,L) <= Tair(i) <= TBounds(3,U), T2(i) == 5/3*Tair(i) - 2/3]);
    constr = [constr, ...
              T2_1, T2_2, T2_3, ...
              sum(zT2(:,i)) == 1]; 
end

% constrain qTD to environment to use implies
constr = [constr, ...
          qTD(1,1:nHops+1) <= 2, ...
          qTD(1,1:nHops+1) >= -2, ...
          qTD(2,1:nHops+1) <= 4, ...
          qTD(2,1:nHops+1) >= -1, ...
          qTD(3,1:nHops+1) <= 1, ...
          qTD(3,1:nHops+1) >= -1, ...
          qTD(4,1:nHops+1) >= -pi/4, ...
          qTD(4,1:nHops+1) <= pi/4, ...
          qTD(5,1:nHops+1) >= -pi/4, ...
          qTD(5,1:nHops+1) <= pi/4, ...
          qTD(6,1:nHops+1) >= -pi/4, ...
          qTD(6,1:nHops+1) <= pi/4];
          
% constrain qCOM to use big M notation
for i = 1:nHops
    for j = 1:nPoints
        constr = [constr, ...
                  qCOM(:,j,i) >= [-2; -1; 0], ...
                  qCOM(:,j,i) <= [2; 4; 0.4]];
    end
end

% constrain qdotTO to use implies
for i = 1:nHops
    constr = [constr, ...
              qdotTO(:,i) >= [-5; -5; 0; -10; -10; -10], ...
              qdotTO(:,i) <= [5; 5; 5; 10; 10; 10]];
end

% kinematic flight time constraints
for i = 1:nHops
    constr = [constr, ...
              qTD(:,i+1) == qTD(:,i) + qdotTO(:,i)*Tair(i) + gravity * T2(i) / 2];
          
    for j = 1:nPoints-2
        constr = [constr, ...
                  qCOM(:,j+1,i) == qCOM(:,1,i) + qdotTO(1:3,i)*j*Tair(i)/(nPoints-1) + gravity(1:3)*(j/(nPoints-1))^2 * T2(i) / 2];
    end
end

% piecewise sinusoids
for i = 1:nHops+1
    % constrain yaw [-2*pi,2*pi]
    constr = [constr, ...
              qTD(6,i) >= -2*pi, ....
              qTD(6,i) <= 2*pi];
    % constrain C and S to [-1,1]
    constr = [constr, ...
              S(i) <= 1, ...
              S(i) >= -1, ...
              C(i) <= 1, ...
              C(i) >= -1];
    % piecewise cos
    C1 = implies(zC(1,i),[CYawBounds(1,L) <= qTD(6,i) <= CYawBounds(1,U), C(i) == 1]);
    C2 = implies(zC(2,i),[CYawBounds(2,L) <= qTD(6,i) <= CYawBounds(2,U), C(i) == -8/(3*pi)*qTD(6,i) - 4]);
    C3 = implies(zC(3,i),[CYawBounds(3,L) <= qTD(6,i) <= CYawBounds(3,U), C(i) == -1]);
    C4 = implies(zC(4,i),[CYawBounds(4,L) <= qTD(6,i) <= CYawBounds(4,U), C(i) == 8/(3*pi)*qTD(6,i) + 4/3]);
    C5 = implies(zC(5,i),[CYawBounds(5,L) <= qTD(6,i) <= CYawBounds(5,U), C(i) == 1]);
    C6 = implies(zC(6,i),[CYawBounds(6,L) <= qTD(6,i) <= CYawBounds(6,U), C(i) == -8/(3*pi)*qTD(6,i) + 4/3]);
    C7 = implies(zC(7,i),[CYawBounds(7,L) <= qTD(6,i) <= CYawBounds(7,U), C(i) == -1]);
    C8 = implies(zC(8,i),[CYawBounds(8,L) <= qTD(6,i) <= CYawBounds(8,U), C(i) == 8/(3*pi)*qTD(6,i) - 4]);
    C9 = implies(zC(9,i),[CYawBounds(9,L) <= qTD(6,i) <= CYawBounds(9,U), C(i) == 1]);
    constr = [constr, ...
              C1, C2, C3, C4, C5, C6, C7, C8, C9, ...
              sum(zC(:,i)) == 1];
    % piecewise sin
    S1 = implies(zS(1,i),[SYawBounds(1,L) <= qTD(6,i) <= SYawBounds(1,U), S(i) == 8/(3*pi)*qTD(6,i)]);
    S2 = implies(zS(2,i),[SYawBounds(2,L) <= qTD(6,i) <= SYawBounds(2,U), S(i) == 1]);
    S3 = implies(zS(3,i),[SYawBounds(3,L) <= qTD(6,i) <= SYawBounds(3,U), S(i) == -8/(3*pi)*qTD(6,i) - 8/3]);
    S4 = implies(zS(4,i),[SYawBounds(4,L) <= qTD(6,i) <= SYawBounds(4,U), S(i) == -1]);
    S5 = implies(zS(5,i),[SYawBounds(5,L) <= qTD(6,i) <= SYawBounds(5,U), S(i) == 8/(3*pi)*qTD(6,i)]);
    S6 = implies(zS(6,i),[SYawBounds(6,L) <= qTD(6,i) <= SYawBounds(6,U), S(i) == 1]);
    S7 = implies(zS(7,i),[SYawBounds(7,L) <= qTD(6,i) <= SYawBounds(7,U), S(i) == -8/(3*pi)*qTD(6,i) + 8/3]);
    S8 = implies(zS(8,i),[SYawBounds(8,L) <= qTD(6,i) <= SYawBounds(8,U), S(i) == -1]);
    S9 = implies(zS(9,i),[SYawBounds(9,L) <= qTD(6,i) <= SYawBounds(9,U), S(i) == 8/(3*pi)*qTD(6,i) - 16/3]);
    constr = [constr, ...
              S1, S2, S3, S4, S5, S6, S7, S8, S9, ...
              sum(zS(:,i)) == 1];
end

% dynamics
for i = 1:nHops+1
    
    Rz = [C(i), -S(i), 0;
          S(i),  C(i), 0;
          0,     0,    1];
    
    % discrete time system dynamics
    if i == 1
        constr = [constr, ...
                  qdotTO(1:3,i) == h1 * (inv(M) * Rz * fnet(:,i) + gravity(1:3)), ...
                  qdotTO(4:6,i) == h1 * inv(I) * (Rz * n(:,i))];
    else
        constr = [constr, ...
                  qdotTO(1:3,i) == qdotTO(1:3,i-1) + gravity(1:3)*Tair(i-1)+ h * (inv(M) * Rz * fnet(:,i) + gravity(1:3)),...
                  qdotTO(4:6,i) == qdotTO(4:6,i-1) + h * inv(I) * (Rz * n(:,i))];
    end
    
    if i ~= nHops+1
        constr = [constr, ...
                  implies(zST(i), Tair(i) == 0)];
    end
    
    constr = [constr, ...
              A_FWP*[fnet(:,i); n(:,i)] <= b_FWP, ...
              fnet(1,i) <= 4*mu/sqrt(2)*fzMAX, ...
              -fnet(1,i) <= 4*mu/sqrt(2)*fzMAX, ...
              fnet(2,i) <= 4*mu/sqrt(2)*fzMAX, ...
              -fnet(2,i) <= 4*mu/sqrt(2)*fzMAX, ...
              0 <= fnet(3,i) <= 4*fzMAX, ...
              10*[-4 -4 -4]' <= n(:,i) <= 10*[4 4 4]'
              ];
end
          
% platform constraints
for i = 1:nPlatforms
    for j = 1:nHops
        Z = implies(1-zTD(i,j),qTD(3,j) == platforms(i,5));
        constr = [constr,...
                  qTD(1,j) >= platforms(i,1) + safety - bigM*zTD(i,j), ... 
                  qTD(1,j) <= platforms(i,2) - safety + bigM*zTD(i,j), ... 
                  qTD(2,j) >= platforms(i,3) + safety - bigM*zTD(i,j), ... 
                  qTD(2,j) <= platforms(i,4) - safety + bigM*zTD(i,j), ...
                  Z];
    end
end

% configuration constraint for flat platforms
for j = 1:nHops
    constr = [constr, ...
              qCOM(:,1,j) == qTD(1:3,j) + COM,...
              qTD(4,j) == 0, ...
              qTD(5,j) == 0];
end

% general configuration constraint
for j = 2:nHops
    constr = [constr, ...
              qCOM(:,1,j) == qCOM(:,end,j-1)];
end
    
for i = 1:nHops
    constr = [constr,...
              sum(zTD(:,i)) == nPlatforms-1];
end

constr = [constr, ...
          zTD(3,2) == 0];
      
constr = [constr, ...
          0.4 <= Tair(:) <= 0.5, ...
          qTD(6,nHops+1) == -0.25];

% objective (minimize impulse, maximimze stance time)
A = 1;
B = 0;
D = 1;
objective = 0;

for i = 1:nHops
    objective = objective + A*(abs(fnet(1,i)) + (abs(fnet(2,i))));
    objective = objective + B*zST(i);
    objective = objective + D*(abs(n(3,i)));
end
objective = objective + qTD(6,nHops+1);

options = sdpsettings('verbose',2,'solver','GUROBI','debug',1);
sol = optimize(constr,objective,options);

%% plot

plot3(value(qTD(1,:)), value(qTD(2,:)), value(qTD(3,:)),'b*');
hold on

zCON = zeros(3,nHops+1);
zCON(1,:) = ones(1,nHops+1);

force_scale = 0.01;
f = GetContactForcesFWP(value(fnet), value(n), zCON);
for i = 1:nHops+1
    for j = 1:nFeet
        quiver3(value(qTD(1,i)) + value(C(i))*rNominal(j,1) + -value(S(i))*rNominal(j,2), ...
                value(qTD(2,i)) + value(S(i))*rNominal(j,1) + value(C(i))*rNominal(j,2), ...
                value(qTD(3,i)) + rNominal(j,3), ...
                force_scale*(value(C(i))*value(f(1,j,i)) + -value(S(i))*value(f(2,j,i))), ...
                force_scale*(value(S(i))*value(f(1,j,i)) + value(C(i))*value(f(2,j,i))), ...
                force_scale*value(f(3,j,i)), 'k');
        hold on
    end
end

for i = 1:nHops
    plot3(value(qCOM(1,:,i)), value(qCOM(2,:,i)), value(qCOM(3,:,i)),'g*-');
    hold on
end

axis equal

%% Save Trajectory in Format for NLP

total_time = h1 + h*nHops + sum(value(Tair(:))); % stance + flight time
time = 0:0.01:total_time;
if time(end) < total_time
    time(end+1) = time(end)+0.01;
end

t_list = [];
t_list = [t_list, 0];
x_list = [];
x_list = [x_list, value([qTD(4:6,1);qTD(1:3,1)+COM;zeros(6,1)])];

for j = 1:nHops+1
    if j ~= 1
        t_list(end+1) = t_list(end) + value(Tair(j-1)) * 1/(nPoints-1); % stance time
        pos = value([qTD(1,j),qTD(2,j),qTD(3,j)]') + COM;
        rpy = value([qTD(4,j),qTD(5,j),qTD(6,j)]');
        vel = value(qdotTO(1:3,j)); 
        rot_vel = value(qdotTO(4:6,j));
        x_list(:,end+1) = [rpy;pos;rot_vel;vel];
    end
    
    if j == 1
        t_list(end+1) = t_list(end) + h1; % stance time
    else
        t_list(end+1) = t_list(end) + h; % stance time
    end
    pos= value([qTD(1,j),qTD(2,j),qTD(3,j)]') + COM;
    rpy= value([qTD(4,j),qTD(5,j),qTD(6,j)]');
    vel= value(qdotTO(1:3,j)); 
    rot_vel= value(qdotTO(4:6,j));
    x_list(:,end+1) = [rpy;pos;rot_vel;vel];
    
    if(j <= nHops)
        if(value(Tair(j)) ~= 0)
            for k = 2:nPoints
                t_list(end+1) = t_list(end) + value(Tair(j)) * 1/(nPoints-1);
                pos = value(qCOM(1:3,k,j));
                rpy = x_list(1:3,end) + 1/(nPoints-1)* ...
                                        (value([qTD(4,j+1),qTD(5,j+1),qTD(6,j+1)]') ...
                                        - value([qTD(4,j),qTD(5,j),qTD(6,j)]'));
                vel = value(qdotTO(1:3,j)) + gravity(1:3)*value(Tair(j)) * (k-1)/(nPoints); 
                rot_vel= value(qdotTO(4:6,j));
                x_list(:,end+1) = [rpy;pos;rot_vel;vel];
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Interpolation to combine data acquired at different frequency rates:
xd = interp1(t_list,x_list',time(1:end-1))';
pos = value(qCOM(1:3,nPoints,nHops));
rpy = value([qTD(4,nHops+1),qTD(5,nHops+1),qTD(6,nHops+1)]');
vel= value(qdotTO(1:3,nHops+1)); 
rot_vel= value(qdotTO(4:6,nHops+1));
x_list(:,end+1) = [rpy;pos;rot_vel;vel];

%% u, ctacts, pfd

pfd = [];
ctacts = [];
ud = [];

for i = 1:size(xd,2)-1
    for j = 1:nHops+1
        if xd(4:6,i) == value(qTD(1:3,j)+COM) % ignoring platform rotation for now
        
        % u and pf to world coordinates
        Rz = [ cos(xd(3,i)),  -sin(xd(3,i)), 0;
               sin(xd(3,i)),   cos(xd(3,i)), 0;
               0,             0,           1];
        Ry = [ cos(xd(2,i)), 0,  sin(xd(2,i));
               0,           1,  0;
              -sin(xd(2,i)), 0,  cos(xd(2,i))];
        Rx = [1, 0,            0;
              0, cos(xd(1,i)), -sin(xd(1,i));
              0, sin(xd(1,i)),  cos(xd(1,i))];
        
        ud = [ud, value(reshape(Rz*Ry*Rx*f(:,:,j),[12,1]))];
        ctacts = [ctacts,ones(4,1)];
        pfd = [pfd, value(reshape(Rz*Ry*Rx*rNominal',[12 1]))+repmat(value(qTD(1:3,j)),4,1)];
        end
    end
    if size(ud,2) < i
        ud = [ud, zeros(12,1)];
        ctacts = [ctacts,zeros(4,1)];
        pfd = [pfd, zeros(12,1)];
    end
end

%% save trajectory
platforms = platforms_true;
save('DiamondHardwareTrajectoryForNLP.mat','xd','ud','pfd','ctacts','platforms')