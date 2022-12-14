function [hybridRef, ctactSeq] = process_high_level_data(option, gait)
% This function parse the high-level planned trajectory to construct a
% multi-phase trajecotry optimization problem to be solved using HSDDP. The
% high-level plan is generated by simulating a RL policy. The RL policy
% used here is from Google's motion imitation work. Details of this work
% could be found at https://github.com/heli-sudoo/motion_imitation

% hybridRef: 1xn cell array of reference trajectory (in structure TrajReference)
% ctactSeq: 1x(n+1) cell array of contact status, each is a 1x4 vector
% dt: integration time step

opts = fixedWidthImportOptions('VariableWidths', 10, 'VariableTypes', 'double');
time_array = readmatrix(sprintf('timestep_%s.txt', gait), opts);
ctact_array = readmatrix(sprintf('contact_%s.txt', gait)); % contact status of each foot at every time step
torque_array = readmatrix(sprintf('torque_%s.txt', gait));
q_array = readmatrix(sprintf('generalized_joint_%s.txt', gait));
qd_array = readmatrix(sprintf('generalized_vel_%s.txt', gait));

%% Set abad angles to zeros
q_array(:,LINKID.abad) = 0;
qd_array(:,LINKID.abad) = 0;
q_array(:,2) = 0;
qd_array(:,2) = 0;
q_array(:,3) = q_array(:,3) - 0.02;
for k = 1:size(q_array, 1)
    qd_array(k,1) = min([qd_array(k,1), 1.0]);
    q_array(k,3) = min([q_array(k,3), 0.28]);
end
q_array(:,[4,5,6]) = 0;
qd_array(:,3) = 0;
qd_array(:,[4 5,6]) = 0;
state_array = [q_array, qd_array];

%% Separate the phases based on contact status
len_horizon = size(ctact_array, 1);
dt = time_array(2) - time_array(1);
ctactSeq = cell(1, 100);
jstart = zeros(1, 100);
jend = zeros(1, 100);
n = 1;
i = 1;
for k = 1:min(len_horizon,200)
    if isempty(ctactSeq{1})
        % If ctactSeq is empty, add the first element of ctact_array
        ctactSeq{i} = ctact_array(1, :);
        jstart(i) = 1;
        jend(i) = 1;
    else
        if ~isequal(ctact_array(k, :), ctactSeq{i})
            ctactSeq{i+1} = ctact_array(k, :);
            jstart(i+1) = jend(i);
            i = i + 1;
        end
        jend(i) = k;
    end
    n = i;
end
ctactSeq(n+1:end) = [];
jstart(n+1:end) = [];
jend(n+1:end) = [];

hybridRef = cell(1, n-1);
for i = 1:n-1
    len_horizon = jend(i)-jstart(i)+1;
    r = TrajReference(36, 12, 12, len_horizon);
    r.xd = mat2cell(state_array(jstart(i):jend(i), :)', 36, ones(1,len_horizon));
    r.ud = mat2cell(torque_array(jstart(i):jend(i), :)', 12, ones(1,len_horizon));
    hybridRef{i} = r;
end

% Append the beginging of the current phase to the end of previous phase
for i = 2:length(hybridRef)
    hybridRef{i-1}.xd = [hybridRef{i-1}.xd, hybridRef{i}.xd{1}]; 
    hybridRef{i-1}.ud = [hybridRef{i-1}.ud, hybridRef{i}.ud{1}];
    hybridRef{i-1}.yd = [hybridRef{i-1}.yd, hybridRef{i}.yd{1}];
    hybridRef{i-1}.len = hybridRef{i-1}.len + 1;
end
%% Flip hip and knee rotations
for i=1:length(hybridRef)
    hybridRef{i} = flip_signs_for_MC(hybridRef{i});
end
%% Filter out short contact
if option.filter_short_contact
    [hybridRef, ctactSeq] = filter_short_contact(hybridRef, ctactSeq);
    plot_phase_duration(hybridRef);
end
%% Refine the trajectory such that time step is dt s
if option.refine_trajectory
    hybridRef = refine_trajectory(hybridRef,option.dt);
end
hybridRef = cellfun(@wbref2kdef, hybridRef, 'UniformOutput',false);
dt = option.dt;
%% Save data to csv file
if option.save_to_file
    % write contact information to csv file
    ctact_fname = 'RolloutTrajectory/contact_post.csv';
    ctact_fid = fopen(ctact_fname, 'w');    
    endidx = -1;
    endTime = 0.0;
    fprintf(ctact_fid, '%s,%s,%s,%s,%s,%s,%s,%s',...
                        'FR', 'RL', 'HR', 'HL', 'startTime', 'endTime',...
                        'startIdx, endIdx');
    fprintf(ctact_fid, '\n');
    for i = 1:min([25,length(hybridRef)])        
        startidx = endidx + 1;
        endidx = startidx + hybridRef{i}.len - 1;
        startTime = endTime;
        endTime = startTime + dt*(hybridRef{i}.len-1);
        fprintf(ctact_fid, '%d,%d,%d,%d,', ctactSeq{i});
        fprintf(ctact_fid, '%6.3f,%6.3f,', startTime, endTime);
        fprintf(ctact_fid, '%d,%d \n', startidx, endidx);
    end
    fclose(ctact_fid);    
    % write state information to csv file
    state_fname = 'RolloutTrajectory/state_post.csv';
    for i = 1:min([25,length(hybridRef)])   
        if i == 1
            writematrix(cell2mat(hybridRef{i}.xd)', state_fname);
        else
            writematrix(cell2mat(hybridRef{i}.xd)', state_fname, 'WriteMode','append');
        end
    end
end
end

function plot_phase_duration(hybridT)
n = length(hybridT);
len_horizons = zeros(1, n);
for i = 1:length(hybridT)
    len_horizons(i) = hybridT{i}.len;
end
figure
plot(1:n, len_horizons);
xlabel('phase index');
ylabel('phase horizon');
end

function hybrid_fine = refine_trajectory(hybridR, dt)
% Refine the trajectory so that the time step is dt s
% Original trajecotry has time step 0.033 s
hybrid_fine = cell(1, length(hybridR));
for i = 1:length(hybrid_fine)
    dur = (hybridR{i}.len-1) * 0.033;
    len_fine = floor(dur/dt) + 1;
    hybrid_fine{i} = TrajReference(36, 12, 12, len_fine);
    Nseg = floor(0.033/dt) + 1;
    k = 1;
    for j = 1:hybridR{i}.len-1
        xbeg = hybridR{i}.xd{j};
        xend = hybridR{i}.xd{j+1};
        x_seg = interpolate_state(xbeg, xend, Nseg);
        hybrid_fine{i}.xd(k:k+Nseg-1) = mat2cell(x_seg, 36, ones(1,Nseg));
        k = k + Nseg - 1;
    end     
end
end

function X = interpolate_state(x1, x2, n)
% x1 -> state at begining
% x2 -> state in the end
% n -> number of interpolated states (including end points)
qs = length(x1)/2;
q1 = x1(1:qs);
q2 = x2(1:qs);
q1d = x1(qs+1:end);

qd = repmat(q1d, [1, n]); % zero-order hold
q = zeros(qs, n); % first-order
for i = 1:qs
    q(i, :) = linspace(q1(i), q2(i), n);
end
X = [q; qd];
end