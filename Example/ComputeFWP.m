clear
clc
close all

addpath([pwd() '/Visualization']);
addpath([pwd(), '/utils']);
addpath(genpath([pwd() '/spatial_v2_extended']));
addpath(genpath([pwd() '/Models']))

% https://arxiv.org/pdf/1712.02731.pdf

%% robot specs

params = getMiniCheetahParams();
robot_tree = buildTreeModel(params);
pos = [0,0,0.18]';
eul = zeros(3,1);
qleg = [0,-1.04049327787845,2.22044586943330];

rNominal = zeros(4,3);

% [fr;fl;hr;hl]
for leg = 1:4
    rNominal(leg,:) = forward_kinematics(pos, eul, qleg', leg);
end

COM = [0 0 0.18]'; % COM position relative to "contact point"
r2COM = zeros(4,length(COM));
for leg = 1:4
    r2COM(leg,:) = rNominal(leg,:) - COM';
end

mu = 0.7;

% compute intersection of AWP and CWC in 3D for each foot, 
% then use vertices to go to 6D, 
% and finally Minkowski sum each foot

%% intersection of AFP and CWC
% CWC - Contact Wrench Cone
A = [ 1  0 -mu/sqrt(2);
     -1  0 -mu/sqrt(2);
      0  1 -mu/sqrt(2);
      0 -1 -mu/sqrt(2);
      0  0  1
      0  0 -1];
    

b = [0;
     0;
     0;
     0;
     150;
     0];
  
FrictionCone = Polyhedron('A', A, 'b', b);
figure(1)
FrictionCone.plot
title('Friction Cone, mu = 0.7 & fz_(max) = 150 N')
xlabel('fx')
ylabel('fy')
zlabel('fz')

% AFP - Actuation Force Polytope
n_motors = 3;
tau_max = 17; % N/m

AFPFR_v = [] ;

for i = 1:2*n_motors
    J = compute_foot_jacobian(robot_tree,pos,eul,qleg',1);
    tau = zeros(n_motors,1);
    if mod(i,2) == 1
        tau(floor(i/2)+1) = tau_max;
    else
        tau(floor(i/2)) = -tau_max;
    end 
    AFPFR_v(end+1,:) = pinv(J(:,7:9)')*tau;
end

AFPFR = Polyhedron('V', AFPFR_v);

figure(2)
subplot(2,2,2)
AFPFR.plot
hold on
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('AFP for Front Right Leg')
axis square

AFPFR = AFPFR.minHRep();

A = [AFPFR.A;FrictionCone.A];
b = [AFPFR.b;FrictionCone.b];

FFPFR = Polyhedron('A',A,'b',b);
FFPFR.minHRep();

figure(3)
subplot(2,2,2)
FFPFR.plot;
hold on
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('FFP for Front Right Leg')
axis square


AFPFL_v = zeros(2*n_motors,3); 

for i = 1:2*n_motors
    J = compute_foot_jacobian(robot_tree,pos,eul,qleg',2);
    tau = zeros(n_motors,1);
    if mod(i,2) == 1
        tau(floor(i/2)+1) = tau_max;
    else
        tau(floor(i/2)) = -tau_max;
    end
    AFPFL_v(i,:) = pinv(J(:,10:12)')*tau;
end

AFPFL = Polyhedron('V', AFPFL_v);

figure(2)
subplot(2,2,1)
AFPFL.plot
hold on
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('AFP for Front Left Leg')
axis square

AFPFL = AFPFL.minHRep();

A = [AFPFL.A;FrictionCone.A];
b = [AFPFL.b;FrictionCone.b];

FFPFL = Polyhedron('A',A,'b',b);
FFPFL.minHRep();

figure(3)
subplot(2,2,1)
FFPFL.plot;
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('FFP for Front Left Leg')
axis square


AFPHR_v = ones(2*n_motors,3); 

for i = 1:2*n_motors
    J = compute_foot_jacobian(robot_tree,pos,eul,qleg',3);
    tau = zeros(n_motors,1);
    if mod(i,2) == 1
        tau(floor(i/2)+1) = tau_max;
    else
        tau(floor(i/2)) = -tau_max;
    end
    AFPHR_v(i,:) = pinv(J(:,13:15)')*tau;
end

AFPHR = Polyhedron('V', AFPHR_v);

figure(2)
subplot(2,2,4)
AFPHR.plot
hold on
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('AFP for Hind Right Leg')
axis square

AFPHR = AFPHR.minHRep();

A = [AFPHR.A;FrictionCone.A];
b = [AFPHR.b;FrictionCone.b];

FFPHR = Polyhedron('A',A,'b',b);
FFPHR.minHRep();

figure(3)
subplot(2,2,4)
FFPHR.plot;
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('FFP for Hind Right Leg')
axis square


AFPHL_v = ones(2*n_motors,3); 

for i = 1:2*n_motors
    J = compute_foot_jacobian(robot_tree,pos,eul,qleg',4);
    tau = zeros(n_motors,1);
    if mod(i,2) == 1
        tau(floor(i/2)+1) = tau_max;
    else
        tau(floor(i/2)) = -tau_max;
    end
    AFPHL_v(i,:) = pinv(J(:,16:18)')*tau;
end

AFPHL = Polyhedron('V', AFPHL_v);

figure(2)
subplot(2,2,3)
AFPHL.plot
hold on
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('AFP for Hind Left Leg')
axis square

AFPHL = AFPHL.minHRep();

A = [AFPHL.A;FrictionCone.A];
b = [AFPHL.b;FrictionCone.b];

FFPHL = Polyhedron('A',A,'b',b);
FFPHL.minHRep();

figure(3)
subplot(2,2,3)
FFPHL.plot;
xlabel('fx')
ylabel('fy')
zlabel('fz')
title('FFP for Hind Left Leg')
axis square

%% FFP to FWP (3D --> 6D)

FFPFR = FFPFR.minVRep();
VFR = zeros(length(FFPFR.V(:,1)),6);
VFR(1:length(FFPFR.V(:,1)),1:length(FFPFR.V(1,:))) = FFPFR.V;

FFPFL = FFPFL.minVRep();
VFL = zeros(length(FFPFL.V(:,1)),6);
VFL(1:length(FFPFL.V(:,1)),1:length(FFPFL.V(1,:))) = FFPFL.V;

FFPHR = FFPHR.minVRep();
VHR = zeros(length(FFPHR.V(:,1)),6);
VHR(1:length(FFPHR.V(:,1)),1:length(FFPHR.V(1,:))) = FFPHR.V;

FFPHL = FFPHL.minVRep();
VHL = zeros(length(FFPHL.V(:,1)),6);
VHL(1:length(FFPHL.V(:,1)),1:length(FFPHL.V(1,:))) = FFPHL.V;

% http://groups.csail.mit.edu/robotics-center/public_papers/Dai16.pdf
for i = 1:length(VFR(:,1))
    VFR(i,4:6) = cross(r2COM(1,:)',VFR(i,1:3));
    VFL(i,4:6) = cross(r2COM(2,:)',VFL(i,1:3));
    VHR(i,4:6) = cross(r2COM(3,:)',VHR(i,1:3));
    VHL(i,4:6) = cross(r2COM(4,:)',VHL(i,1:3));
end

FWPFR = Polyhedron(VFR);
FWPFL = Polyhedron(VFL);
FWPHR = Polyhedron(VHR);
FWPHL = Polyhedron(VHL);

%% save FWPs

FWPFR = FWPFR.minVRep();
V_FWPFR = FWPFR.V;

FWPFL = FWPFL.minVRep();
V_FWPFL = FWPFL.V;

FWPHR = FWPHR.minVRep();
V_FWPHR = FWPHR.V;

FWPHL = FWPHL.minVRep();
V_FWPHL = FWPHL.V;

save("FWP_each_foot",'V_FWPFR','V_FWPFL','V_FWPHR','V_FWPHL');

%% Minkowski sum for full FWP

FWP = plus(FWPFR,FWPFL);
FWP = plus(FWP,FWPHR);
FWP = plus(FWP,FWPHL);

FWP = FWP.minHRep();

%% save FWP 
% not sure why, but saving vertices is much more reliable

V_FWP = FWP.V;
save("FWP_Mini_Cheetah_mu_07.mat",'V_FWP');

%% plot FFP for visualizatoin

FFP = plus(FFPFR,FFPFL);
FFP = plus(FFP,FFPHR);
FFP = plus(FFP,FFPHL);

figure(4)
FFP.plot
xlabel('fnetx')
ylabel('fnety')
zlabel('fnetz')
title('FFP')
axis square

%% plot FNP for visualization

VFR = zeros(length(FFPFR.V(:,1)),3);
VFR(1:length(FFPFR.V(:,1)),1:length(FFPFR.V(1,:))) = FFPFR.V;

VFL = zeros(length(FFPFL.V(:,1)),3);
VFL(1:length(FFPFL.V(:,1)),1:length(FFPFL.V(1,:))) = FFPFL.V;

VHR = zeros(length(FFPHR.V(:,1)),3);
VHR(1:length(FFPHR.V(:,1)),1:length(FFPHR.V(1,:))) = FFPHR.V;

VHL = zeros(length(FFPHL.V(:,1)),3);
VHL(1:length(FFPHL.V(:,1)),1:length(FFPHL.V(1,:))) = FFPHL.V;

for i = 1:length(VFR(:,1))
    VFR(i,:) = cross(r2COM(1,:)',VFR(i,1:3));
    VFL(i,:) = cross(r2COM(2,:)',VFL(i,1:3));
    VHR(i,:) = cross(r2COM(3,:)',VHR(i,1:3));
    VHL(i,:) = cross(r2COM(4,:)',VHL(i,1:3));
end

FNPFR = Polyhedron('V',VFR);
FNPFL = Polyhedron('V',VFL);
FNPHR = Polyhedron('V',VHR);
FNPHL = Polyhedron('V',VHL);

figure(5)
subplot(2,2,1)
FNPFR.plot
xlabel('nx')
ylabel('ny')
zlabel('nz')
title('FNP for Front Right Leg')
axis square

subplot(2,2,2)
FNPFL.plot
xlabel('nx')
ylabel('ny')
zlabel('nz')
title('FNP for Front Left Leg')
axis square

subplot(2,2,3)
FNPHR.plot
xlabel('nx')
ylabel('ny')
zlabel('nz')
title('FNP for Hind Right Leg')
axis square

subplot(2,2,4)
FNPHL.plot
xlabel('nx')
ylabel('ny')
zlabel('nz')
title('FNP for Hind Left Leg')
axis square

FNP = plus(FNPFR,FNPFL);
FNP = plus(FNP,FNPHR);
FNP = plus(FNP,FNPHL);

figure(6)
FNP.plot
xlabel('nx')
ylabel('ny')
zlabel('nz')
title('FNP')
axis square

%% save FFP for each leg

save("FFP.mat",'FFPFR','FFPFL','FFPHR','FFPHL');

                 
                 

                 
                 
 




