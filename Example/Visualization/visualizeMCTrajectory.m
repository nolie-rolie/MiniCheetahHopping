function q = visualizeMCTrajectory(data, option, varargin)
% eul-> ZYX euler anlges 3xn matrix
% pos-> Position of CoM 3xn matrix
% qdummy-> represents either joint angle or foot placement
robot = importrobot("mini_cheetah_simple_v2.urdf");
% robot = importrobot("mini_cheetah_mesh.urdf");
robot = float_base_quadruped(robot);
robot.DataFormat = 'column';
params = getMiniCheetahParams();
legIK = LegIK();

time = data.time;
eul = data.eul(:, time);
eul = [0,0,1;0,1,0;1,0,0]*eul;
pos = data.pos(:,time);
ctacts = data.ctacts(:, time);
qdummy = data.qdummy(:, time);
% initialize joint angles with qdummy
qJ = qdummy;
pf = qdummy;
for i = 1:length(time)
    c = ctacts(:,i);
    Rbody = eul2Rot(eul(:,i));
    for l = 1:4
        if c(l)==1 % if in stance
            pf_leg = qdummy(3*(l-1)+1:3*l, i);
            prel = Rbody'*(pf_leg - pos(:, i));
            qJleg = legIK.solve(prel, l);
            qJ(3*(l-1)+1:3*l, i) = qJleg;
        end
        if c(l)==0 % if in swing
            pf(3*(l-1)+1:3*l, i) = 0;
        end
    end
end

qJ_init = repmat([0,-0.7696,1.6114],1,4)';

for i = 1:length(time)-1
    for l = 1:4
       if isequal(qJ(3*(l-1)+1:3*l,i),zeros(3,1))
           j = 1;
           while i+j < length(time) && isequal(qJ(3*(l-1)+1:3*l,i+j),zeros(3,1))
               j = j + 1;
           end
           if i+j == length(time)
               for k = 1:j
                   qJ(3*(l-1)+1:3*l,i+k-1) = qJ(3*(l-1)+1:3*l,i-1) + k/j * (qJ_init(3*(l-1)+1:3*l)-qJ(3*(l-1)+1:3*l,i-1));
               end
               qJ(3*(l-1)+1:3*l,i+j) = qJ_init(3*(l-1)+1:3*l);
           elseif i == 1
               for k = 1:j
                   qJ(3*(l-1)+1:3*l,i+k-1) = qJ_init(3*(l-1)+1:3*l) + k/j * (qJ(3*(l-1)+1:3*l,i+j)-qJ_init(3*(l-1)+1:3*l));
               end
           else
               for k = 1:j
                   qJ(3*(l-1)+1:3*l,i+k-1) = qJ(3*(l-1)+1:3*l,i-1) + k/j * (qJ(3*(l-1)+1:3*l,i+j)-qJ(3*(l-1)+1:3*l,i-1));
               end
           end
       end
    end
end

q = [pos; eul; qJ];

if ~isempty(data.F)
    F = data.F(:,time)/60;
end

bigAnim = figure(198);
clf
set(bigAnim,'Renderer','OpenGL');
% set(bigAnim, 'Position', get(0, 'Screensize'));
set(bigAnim, 'Position');

ax = show(robot, q(:,1), "Frames","off","collision","off","PreservePlot",0, "FastUpdate",1);
if option.setCameraPos
    set(ax, 'CameraPosition', data.CameraPos);
else
    set(ax, 'CameraPosition', [-3, -1, 1]);
end
hold on

if option.show_floor
    drawFloor();
end

if option.show_platforms
    drawSquarePlatforms(data.platforms);
end
if option.show_angled_platforms
    drawAngledPlatforms(data.angled_platforms);
end
if option.show_obstacles
    drawObstacles(data.obstacles);
end

if option.show_footloc
    % initialize foot location with transparent ball
    footObjs{1} = drawBall(zeros(1,3),0.02,'g',0);
    footObjs{2} = drawBall(zeros(1,3),0.02,'g',0);
    footObjs{3} = drawBall(zeros(1,3),0.02,'g',0);
    footObjs{4} = drawBall(zeros(1,3),0.02,'g',0);
end
if option.show_GRF
    forceObjs{1} = drawArrow(0.3, 0.01, [1,0,0]);
    forceObjs{2} = drawArrow(0.3, 0.01, [1,0,0]);
    forceObjs{3} = drawArrow(0.3, 0.01, [1,0,0]);
    forceObjs{4} = drawArrow(0.3, 0.01, [1,0,0]);
end
axis(5*[-1 1 -1 1 -1 1])
% axis off
box off
axis equal

if nargin > 2
    titletxt = varargin{1};
    title(ax, titletxt);
end

for k = 1:length(time)
    set(ax, 'CameraTarget', q(1:3, k)');
    show(robot, q(:,k), "Frames","off",'collision','off','PreservePlot',0);
    for leg = 1:4
        if option.show_footloc && (~isempty(pf))
            footObjs{leg} = updateBall(footObjs{leg}, pf(3*(leg-1)+1:3*leg, k), [1,1,1], 0.8);
            if ~isempty(ctacts)
                if ~ctacts(leg, k)
                    hideBall(footObjs{leg});
                end
            end
        end  
        if option.show_GRF && (~isempty(F))  && (~isempty(pf))
            Fk = eul2Rot(eul(:,k))*F(3*(leg-1)+1:3*leg, k);
            updateArrow(forceObjs{leg}, pf(3*(leg-1)+1:3*leg, k), Fk);
            if ~isempty(ctacts)
                if ~ctacts(leg, k)
                    hideObject(forceObjs{leg});
                end
            end
        end
        if ~isempty(ctacts)
            if ctacts(leg, k)
                % remove visuals for leg if in contact
                if option.hide_leg
                    clearVisual(robot.Bodies{6+4*(leg-1)+1});
                    clearVisual(robot.Bodies{6+4*(leg-1)+2});
                    clearVisual(robot.Bodies{6+4*(leg-1)+3});
                    clearVisual(robot.Bodies{6+4*(leg-1)+4});
                end                
            end
        end
    end     
    pause(0.001);
    drawnow
end
if nargout >= 1
    varargout{1} = gca;
end
end