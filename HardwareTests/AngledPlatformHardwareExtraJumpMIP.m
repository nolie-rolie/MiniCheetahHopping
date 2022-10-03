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
COM(3) = 0.1;
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
Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

h1 = 2e-1;
h = 2e-1; % timestep for discrete time system dynamics ... I'm making this up :0
hwait = 6e-1;

m = 9; % mass in kg 
M = diag([m m m]);

radius = norm(r2COM(1,:));
radius_platform = norm(rNominal(1,:));

m = 9; % mass in kg 

radius = norm(r2COM(1,:));
radius_platform = norm(rNominal(1,:));

% friction cone
mu = 0.4;
Af = [ 1  0 -mu/sqrt(2);
      -1  0 -mu/sqrt(2);
       0  1 -mu/sqrt(2);
       0 -1 -mu/sqrt(2);
       0  0  1
       0  0 -1];
    

bf = [0;
      0;
      0;
      0;
      150;
      0];

%% lay out environment

% flat platforms
platform1_xLB = -0.25;
platform1_xUB = 0.25;
platform1_yLB = -0.25;
platform1_yUB = 0.25;
platform1_z= 0;

platform2_xLB = platform1_xUB + 0.2;
platform2_xUB = platform2_xLB+0.7;
platform2_yLB = -1;
platform2_yUB = platform1_yUB;
platform2_z = 0;

platforms = [platform1_xLB platform1_xUB platform1_yLB platform1_yUB platform1_z;
             platform2_xLB platform2_xUB platform2_yLB platform2_yUB platform2_z];

nFlatPlatforms = length(platforms(:,1));

for i = 1:nFlatPlatforms
    platforms(i,1) = platforms(i,1) + 0.5;
    platforms(i,2) = platforms(i,2) + 0.5;
end

platforms = [-0.25, 0.25, platform1_yLB, platform1_yUB, 0;
             platforms];

nFlatPlatforms = nFlatPlatforms + 1;

% angled platforms
angled_platform1_xLB = platforms(2,1);
angled_platform1_xUB = platforms(3,2);
angled_platform1_yLB = platform1_yUB + 0.;
angled_platform1_yUB = angled_platform1_yLB + 1;
angled_platform1_z = 0.15;
angled_platform1_roll = 0.2; % roll angle

R1 = [1 0 0;
      0 cos(angled_platform1_roll) -sin(angled_platform1_roll);
      0 sin(angled_platform1_roll) cos(angled_platform1_roll)];
n1 = R1*COM;

R1=R1';

angled_platforms = [angled_platform1_xLB angled_platform1_xUB ...
                    angled_platform1_yLB angled_platform1_yUB ...
                    angled_platform1_z ...
                    n1(1) n1(2) n1(3)];

nAngledPlatforms = length(angled_platforms(:,1));

% obstacles
obstacle1_xLB = (platforms(3,1) - platforms(2,2))/2 + platforms(2,2);
obstacle1_xUB = (platforms(3,1) - platforms(2,2))/2 + platforms(2,2);
obstacle1_yLB = platform2_yLB;
obstacle1_yUB = platform2_yUB;
obstacle1_zLB = 0;
obstacle1_zUB = 0.5;

obstacles = [obstacle1_xLB obstacle1_xUB obstacle1_yLB obstacle1_yUB obstacle1_zLB obstacle1_zUB];

nObstacles = length(obstacles(:,1));

% wall
[y,z] = meshgrid(obstacle1_yLB:0.1:obstacle1_yUB,0:0.1:0.5);
x = ones(size(y))*obstacle1_xLB;
wall = surf(x,y,z);
set(wall,'FaceColor','black');
hold on

% points
stPoint = [0, 0, platforms(1,5), 0, 0, 0];

%% plot environment

figure(1)
% angled platforms
[x,y] = meshgrid(angled_platform1_xLB:0.1:angled_platform1_xUB,angled_platform1_yLB:0.1:angled_platform1_yUB);
% z = tan(-angled_platform1_roll)*x + angled_platform1_z;
z = angled_platform1_z - n1(2)/n1(3)*(y - (angled_platform1_yUB-angled_platform1_yLB));
s1 = surf(x,y,z);
set(s1,'FaceColor',[0.7,0.7,0.7])
axis square
hold on

% points
plot3([stPoint(1)],[stPoint(2)], [stPoint(3)],'k*')
hold on

% platforms

for i = 1:nFlatPlatforms
    p1 = [platforms(i,1) platforms(i,3) platforms(i,5)];
    p2 = [platforms(i,2) platforms(i,3) platforms(i,5)];
    p3 = [platforms(i,2) platforms(i,4) platforms(i,5)];
    p4 = [platforms(i,1) platforms(i,4) platforms(i,5)]; 

    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
%     
%     if i == 1
%         color = 'gray';
%     else
%         color = 'gray';
%     end
    fill3(x, y, z, [0.7,0.7,0.7]);
    xlabel('x'); ylabel('y'); zlabel('z'); 
    hold on
end


%% adjust platforms width/length to ensure all feet land on platforms

platforms_true = platforms;
for i = 1:nFlatPlatforms
    platforms(i,1) = platforms_true(i,1) + radius_platform;
    platforms(i,2) = platforms_true(i,2) - radius_platform;
    platforms(i,3) = platforms_true(i,3) + radius_platform;
    platforms(i,4) = platforms_true(i,4) - radius_platform;
end
platforms(2,1) = platforms_true(2,1) + radius_platform*2/3;
platforms(2,2) = platforms_true(2,2) - radius_platform*2/3;
platforms(2,3) = platforms_true(2,3) + radius_platform*2/3;
platforms(2,4) = platforms_true(2,4) - radius_platform*2/3;

axis square
axis equal

%% formulate and solve MIP problem
nPoints = 10; % increase to decrease prob of hitting obstacle
nHops = 4; 
nBoundaries = 6;
gravity = [0 0 -9.8 0 0 0]';
bigM = 1e4;
epsilon = 1e-3;
safety = 0;%10e-2;

% state variables
qCOM = sdpvar(3,nPoints,nHops,'full');  % COM position
zCOM = binvar(nBoundaries,nPoints*nHops,nObstacles); % binaries to restrict COM position

qTD = sdpvar(6,nHops+1,'full');    % Contact points
zTD = binvar(nFlatPlatforms+nAngledPlatforms,nHops,'full'); % binaries to restrict footstep position

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
          qTD(1,1:nHops) <= 5, ...
          qTD(1,1:nHops) >= -5, ...
          qTD(2,1:nHops) <= 5, ...
          qTD(2,1:nHops) >= -5, ...
          qTD(3,1:nHops) <= 2, ...
          qTD(3,1:nHops) >= -1, ...
          qTD(4,1:nHops) >= -pi/4, ...
          qTD(4,1:nHops) <= pi/4, ...
          qTD(5,1:nHops) >= -pi/4, ...
          qTD(5,1:nHops) <= pi/4, ...
          qTD(6,1:nHops+1) >= -pi/8, ...
          qTD(6,1:nHops+1) <= pi/8];
          
% constrain qCOM to use big M notation
for i = 1:nHops
    for j = 1:nPoints
        constr = [constr, ...
                  qCOM(:,j,i) >= [-5; -5; 0], ...
                  qCOM(:,j,i) <= [5; 5; 0.7]];
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
              qTD(4:6,i+1) == qTD(4:6,i) + qdotTO(4:6,i)*Tair(i)];
          
    for j = 1:nPoints-1
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
    % discrete time system dynamics
    if i == 1
        constr = [constr, ...
                  qdotTO(1,i) == h * (1/m*(C(i)*fnet(1,i) + -S(i)*fnet(2,i))), ...
                  qdotTO(2,i) == h * (1/m*(S(i)*fnet(1,i) + C(i)*fnet(2,i))), ...
                  qdotTO(3,i) == h * (1/m*fnet(3,i) + gravity(3)), ...
                  qdotTO(4,i) == h * 1/Ixx *(C(i)*n(1,i) + -S(i)*n(2,i)), ...
                  qdotTO(5,i) == h * 1/Iyy *(S(i)*n(1,i) + C(i)*n(2,i)), ...
                  qdotTO(6,i) == h * 1/Izz *n(3,i)];
    elseif i ~= nHops + 1
        constr = [constr, ...
                  implies(zTD(3,i),qdotTO(1,i) == qdotTO(1,i-1) + h * (1/m*(C(i)*fnet(1,i) + -S(i)*fnet(2,i)) + gravity(1))), ...
                  implies(zTD(3,i),qdotTO(2,i) == qdotTO(2,i-1) + h * (1/m*(S(i)*fnet(1,i) + C(i)*fnet(2,i)) + gravity(2))), ...
                  implies(zTD(3,i),qdotTO(3,i) == (qdotTO(3,i-1) + gravity(3)*Tair(i-1)) + h * (1/m*fnet(3,i) + gravity(3))), ...
                  implies(zTD(3,i),qdotTO(4,i) == qdotTO(4,i-1) + h * 1/Ixx * (C(i)*n(1,i) + -S(i)*n(2,i))), ...
                  implies(zTD(3,i),qdotTO(5,i) == qdotTO(5,i-1) + h * 1/Iyy * (S(i)*n(1,i) + C(i)*n(2,i))), ...
                  implies(zTD(3,i),qdotTO(6,i) == qdotTO(6,i-1) + h * 1/Izz * n(3,i))];
    else
        constr = [constr, ...
                  qdotTO(1,i) == qdotTO(1,i-1) + h * (1/m*(C(i)*fnet(1,i) + -S(i)*fnet(2,i)) + gravity(1)), ...
                  qdotTO(2,i) == qdotTO(2,i-1) + h * (1/m*(S(i)*fnet(1,i) + C(i)*fnet(2,i)) + gravity(2)), ...
                  qdotTO(3,i) == (qdotTO(3,i-1) + gravity(3)*Tair(i-1)) + h * (1/m*fnet(3,i) + gravity(3)), ...
                  qdotTO(4,i) == qdotTO(4,i-1) + h * 1/Ixx * (C(i)*n(1,i) + -S(i)*n(2,i)), ...
                  qdotTO(5,i) == qdotTO(5,i-1) + h * 1/Iyy * (S(i)*n(1,i) + C(i)*n(2,i)), ...
                  qdotTO(6,i) == qdotTO(6,i-1) + h * 1/Izz * n(3,i)];
    end
    
    % angled
    if i ~= 1 && i ~= nHops+1
        constr = [constr, ...
                      implies(1-zTD(3,i),qdotTO(1:3,i) == qdotTO(1:3,i-1) + gravity(1:3)*Tair(i-1) + ...
                                h*(inv(M)*[C(i),-S(i),0;S(i),C(i),0;0,0,1]*R1'* ...
                                fnet(:,i)+gravity(1:3))), ...
                      implies(1-zTD(3,i),qdotTO(4:6,i) == qdotTO(4:6,i-1) + gravity(4:6)*Tair(i-1) + ...
                                h*(inv(I)*[C(i),-S(i),0;S(i),C(i),0;0,0,1]*R1'* ...
                                n(:,i)+gravity(4:6)))];
    end
    
    if i ~= nHops+1
        constr = [constr, ...
                  implies(zST(i), Tair(i) == 0)];
    end
    
    % friction cone & constrain fz to match robot capability
    constr = [constr, ...
              A_FWP*[fnet(:,i); n(:,i)] <= b_FWP, ...
              fnet(1,i) <= 4*mu/sqrt(2)*fzMAX, ...
              -fnet(1,i) <= 4*mu/sqrt(2)*fzMAX, ...
              fnet(2,i) <= 4*mu/sqrt(2)*fzMAX, ...
              -fnet(2,i) <= 4*mu/sqrt(2)*fzMAX, ...
              0 <= fnet(3,i) <= 4*fzMAX, ...
              1e2*[-3 -3 -3]' <= n(:,i) <= 1e2*[3 3 3]', ...
%               Af*fnet(:,i) <= bf
              ];
          
    constr = [constr, ...
              fnet(1,i) <= mu/sqrt(2)*fnet(3,i), ...
              -fnet(1,i) <= mu/sqrt(2)*fnet(3,i), ...
              fnet(2,i) <= mu/sqrt(2)*fnet(3,i), ...
              -fnet(2,i) <= mu/sqrt(2)*fnet(3,i), ...
              0 <= fnet(3,i) <= 4*fzMAX
              ];
end

% obstacle constraints
for i = 1:nObstacles
    for j = 1:nHops
        constr = [constr,...
                  qCOM(1,:,j) <= obstacles(i,1) - radius + zCOM(1,(j-1)*nPoints+1:j*nPoints,i)*bigM, ...
                  qCOM(1,:,j) >= obstacles(i,2) + radius - zCOM(2,(j-1)*nPoints+1:j*nPoints,i)*bigM, ...
                  qCOM(2,:,j) <= obstacles(i,3) - radius + zCOM(3,(j-1)*nPoints+1:j*nPoints,i)*bigM, ...
                  qCOM(2,:,j) >= obstacles(i,4) + radius - zCOM(4,(j-1)*nPoints+1:j*nPoints,i)*bigM, ...
                  qCOM(3,:,j) <= obstacles(i,5) - radius + zCOM(5,(j-1)*nPoints+1:j*nPoints,i)*bigM, ...
                  qCOM(3,:,j) >= obstacles(i,6) + radius - zCOM(6,(j-1)*nPoints+1:j*nPoints,i)*bigM, ...
                  zCOM(1,(j-1)*nPoints+1:j*nPoints,i) + zCOM(2,(j-1)*nPoints+1:j*nPoints,i) + zCOM(3,(j-1)*nPoints+1:j*nPoints,i) ...
                    + zCOM(4,(j-1)*nPoints+1:j*nPoints,i) + zCOM(5,(j-1)*nPoints+1:j*nPoints,i) + zCOM(6,(j-1)*nPoints+1:j*nPoints,i) <= 5];
    end
end
          
% platform constraints
for i = 1:nFlatPlatforms
    Z = implies(1-zTD(i,1:nHops),qTD(3,1:nHops) == platforms(i,5));
    constr = [constr,...
              qTD(1,1:nHops) >= platforms(i,1) + safety - bigM*zTD(i,1:nHops), ... 
              qTD(1,1:nHops) <= platforms(i,2) - safety + bigM*zTD(i,1:nHops), ... 
              qTD(2,1:nHops) >= platforms(i,3) + safety - bigM*zTD(i,1:nHops), ... 
              qTD(2,1:nHops) <= platforms(i,4) - safety + bigM*zTD(i,1:nHops), ...
              Z];
end

% configuration constraint for flat platforms
for j = 1:nHops
    constr = [constr, ...
              implies(zTD(nFlatPlatforms+1,j),qCOM(:,1,j) == qTD(1:3,j) + COM),...
              implies(zTD(nFlatPlatforms+1,j),qTD(4,j) == 0), ...
              implies(zTD(nFlatPlatforms+1,j),qTD(5,j) == 0)];
end

% angled platform
XZ = implies(1-zTD(nFlatPlatforms+1,:),qTD(3,1:nHops) == angled_platform1_z - n1(2)/n1(3)*(qTD(2,1:nHops) - (angled_platform1_yUB-angled_platform1_yLB)));
constr = [constr,...
          qTD(1,1:nHops) >= angled_platform1_xLB + radius_platform + safety - bigM*zTD(nFlatPlatforms+1,:), ... 
          qTD(1,1:nHops) <= angled_platform1_xUB - radius_platform - safety + bigM*zTD(nFlatPlatforms+1,:), ... 
          qTD(2,1:nHops) >= angled_platform1_yLB + radius_platform + safety - bigM*zTD(nFlatPlatforms+1,:), ... 
          qTD(2,1:nHops) <= angled_platform1_yUB - radius_platform - safety + bigM*zTD(nFlatPlatforms+1,:), ...
          XZ];
% configuration constraint for angled platforms
for j = 1:nHops
    constr = [constr, ...
              implies(1-zTD(nFlatPlatforms+1,j),qCOM(:,1,j) == qTD(1:3,j) + n1),...
              implies(1-zTD(nFlatPlatforms+1,j),qTD(4,j) == angled_platform1_roll), ...
              implies(1-zTD(nFlatPlatforms+1,j),qTD(5,j) == 0)];
end

% general configuration constraint
for j = 2:nHops
    constr = [constr, ...
              qCOM(:,1,j) == qCOM(:,end,j-1)];
end
    
for i = 1:nHops
    constr = [constr,...
              sum(zTD(:,i)) == nFlatPlatforms+nAngledPlatforms-1];
end

constr = [constr, ...
          qTD(6,nHops+1) <= 0, ...
% %           Tair(1) >= 0.5, ...
%           0.4 <= Tair(2) <= 0.5, ...
% %           Tair(3) >= 0.4, ...
% %           qTD(2,2) == qTD(2,3), ...
%           0.4 <= qTD(1,3) <= 0.5, ...
%           qTD(1,2) <= 0.1, ...
% % %           qTD(1,3) >= 0.6, ...
%           zTD(3,2:3) == 0
          ];

constr = [constr, ...
          zTD(2,2) == 0,...
          zTD(4,3:4) == 0, ...
          qTD(6,1:nHops) == 0, ...
          Tair(1) == 0.4
          ];

% objective (minimize impulse, maximimze stance time)
A = 0;
B = 0;%-1;
D = 0;
objective = 0;
% objective = qTD(6,2)^2 + qTD(6,3)^2;
objective = - 100*(qTD(1,2) - qTD(2,2));
% linear cost function
for i = 1:nHops
    objective = objective + A*(fnet(3,i)) + B*zST(i);
    objective = objective + fnet(1,i)^2 + fnet(2,i)^2;
    objective = objective + D*zTD(3,i);
    % A = 0 is fast, but just feasible is fastest
    % only minimizing z force takes a few seconds
end

options = sdpsettings('verbose',2,'solver','GUROBI','debug',1);
sol = optimize(constr,objective,options);

qCOM = value(qCOM);
qTD = value(qTD);
save("AngledPlatformMIPSolutionForVisualization",'qCOM','qTD');
%% plot

plot3(value(qTD(1,:)), value(qTD(2,:)), value(qTD(3,:)),'b*');
hold on

zCON = zeros(3, nHops+1);
zCON(1,:) = ones(1,nHops+1);

force_scale = 0.001;
f = GetContactForcesFWP(value(fnet), value(n), zCON);
f(:,:,2) = R1*f(:,:,2);
f(:,:,3) = R1*f(:,:,3);
for i = 1:nHops+1
    for j = 1:nFeet
        quiver3(value(qTD(1,i)) + value(C(i))*rNominal(j,1) + -value(S(i))*rNominal(j,2), ...
                value(qTD(2,i)) + value(S(i))*rNominal(j,1) + value(C(i))*rNominal(j,2), ...
                value(qTD(3,i)) + rNominal(j,3), ...
                force_scale*(value(C(i))*value(f(1,j,i)) + value(S(i))*value(f(2,j,i))), ...
                force_scale*(-value(S(i))*value(f(1,j,i)) + value(C(i))*value(f(2,j,i))), ...
                force_scale*value(f(3,j,i)), 'k');
        hold on
    end
end

for i = 1:nHops
    plot3(value(qCOM(1,:,i)), value(qCOM(2,:,i)), value(qCOM(3,:,i)),'g*-');
    hold on
end

axis equal

noJumps = 0;
for i = 1:nHops
    if value(Tair(i)) == 0
        noJumps = noJumps + 1;
    end
end

h = hwait;
total_time = h1 + h*(nHops - noJumps) + sum(value(Tair(:))); % stance + flight time
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
        if value(Tair(j-1)) ~= 0
            t_list(end+1) = t_list(end) + value(Tair(j-1)) * 1/nPoints; % stance time
            if j < nHops+1
                if value(zTD(3,j)) == 0
                    pos = value([qTD(1,j),qTD(2,j),qTD(3,j)]') + n1;
                else
                    pos = value([qTD(1,j),qTD(2,j),qTD(3,j)]') + COM;
                end
            else
                pos= value([qTD(1,j),qTD(2,j),qTD(3,j)]') + COM;
            end
            rpy = value([qTD(4,j),qTD(5,j),qTD(6,j)]');
            vel = value(qdotTO(1:3,j)); 
            rot_vel = value(qdotTO(4:6,j));
            x_list(:,end+1) = [rpy;pos;rot_vel;vel];
        end
    end
    if j == 1 || value(Tair(j-1)) ~= 0
        
        if j == 1
            t_list(end+1) = t_list(end) + h1; % stance time
        else
            t_list(end+1) = t_list(end) + h; % stance time
        end
        
        if j < nHops+1
            if value(zTD(3,j)) == 0
                pos = value([qTD(1,j),qTD(2,j),qTD(3,j)]') + n1;
            else
                pos = value([qTD(1,j),qTD(2,j),qTD(3,j)]') + COM;
            end
        else
            pos= value([qTD(1,j),qTD(2,j),qTD(3,j)]') + COM;
        end
        
        rpy = value([qTD(4,j),qTD(5,j),qTD(6,j)]');
        vel = value(qdotTO(1:3,j)); 
        rot_vel = value(qdotTO(4:6,j));
        x_list(:,end+1) = [rpy;pos;rot_vel;vel];
        
    end
    
    if(j <= nHops)
        if(value(Tair(j)) ~= 0)
            for k = 2:nPoints
                t_list(end+1) = t_list(end) + value(Tair(j)) * 1/nPoints;
                pos = value(qCOM(1:3,k,j));
                rpy = x_list(1:3,end) + 1/nPoints* ...
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
xd(:,end+1) = [rpy;pos;rot_vel;vel];

%% u, ctacts, pfd

pfd = [];
ctacts = [];
ud = [];

for i = 1:length(xd)-1
    
    Rz = [ cos(xd(3,i)),  -sin(xd(3,i)), 0;
           sin(xd(3,i)),   cos(xd(3,i)), 0;
           0,              0,            1];
    Ry = [ cos(xd(2,i)), 0,  sin(xd(2,i));
           0,            1,   0;
          -sin(xd(2,i)), 0,  cos(xd(2,i))];
    Rx = [1, 0,            0;
          0, cos(xd(1,i)), -sin(xd(1,i));
          0, sin(xd(1,i)),  cos(xd(1,i))];
          
    for j = 1:nHops+1
%         if xd(4:6,i) == value(qTD(1:3,j)+COM)
%             ud = [ud, value(reshape([C(j),S(j),0;-S(j),C(j),0;0,0,1]*f(:,:,j),[12,1]))];
%             ctacts = [ctacts,ones(4,1)];
%             pfd = [pfd, value(reshape([C(j),S(j),0;-S(j),C(j),0;0,0,1]*rNominal',[12 1]))+repmat(value(qTD(1:3,j)),4,1)];
%         elseif xd(4:6,i) == value(qTD(1:3,j)+n1)
%             ud = [ud, value(reshape([C(j),S(j),0;-S(j),C(j),0;0,0,1]*R1*f(:,:,j),[12,1]))];
%             ctacts = [ctacts,ones(4,1)];
%             pfd = [pfd, value(reshape([C(j),S(j),0;-S(j),C(j),0;0,0,1]*R1*rNominal',[12 1]))+repmat(value(qTD(1:3,j)),4,1)];
%         end
        if xd(4:6,i) == value(qTD(1:3,j)+COM)
            ud = [ud, value(reshape(Rz*f(:,:,j),[12,1]))];
            ctacts = [ctacts,ones(4,1)];
            RzFoot = [ cos(qTD(6,j)),  -sin(qTD(6,j)), 0;
                       sin(qTD(6,j)),   cos(qTD(6,j)), 0;
                       0,              0,              1];
            pfd = [pfd, value(reshape(RzFoot*rNominal',[12 1]))+repmat(value(qTD(1:3,j)),4,1)];
        elseif xd(4:6,i) == value(qTD(1:3,j)+n1)
            ud = [ud, value(reshape(Rz*Ry*Rx*f(:,:,j),[12,1]))];
            ctacts = [ctacts,ones(4,1)];
            RzFoot = [ cos(qTD(6,j)),  -sin(qTD(6,j)), 0;
                       sin(qTD(6,j)),   cos(qTD(6,j)), 0;
                       0,              0,              1];
            RxFoot = [1, 0,            0;
                      0, cos(angled_platform1_roll), -sin(angled_platform1_roll);
                      0, sin(angled_platform1_roll),  cos(angled_platform1_roll)];
            pfd = [pfd, value(reshape(RzFoot*RxFoot*rNominal',[12 1]))+repmat(value(qTD(1:3,j)),4,1)];
        end
    end
    if length(ud(1,:)) < i
        ud = [ud, zeros(12,1)];
        ctacts = [ctacts,zeros(4,1)];
        pfd = [pfd, zeros(12,1)];
    end
end

%% save trajectory
platforms = platforms_true;
save('AngledPlatformTrajectoryForNLP.mat','xd','ud','pfd','ctacts','platforms','obstacles','angled_platforms')