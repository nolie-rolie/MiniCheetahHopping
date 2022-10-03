function kdref = wbref2kdef(wbref)
% takes reference trajectory of whole-body state to reference trajectory of
% kinodynamic model state
len_horizon = wbref.len;
kdref = TrajReference(24,24,1,len_horizon);
xd_wb = [wbref.xd{:}];
pos = xd_wb(1:3,:);
eul = rpy2eul(xd_wb(4:6,:));
qJ = xd_wb(7:18,:);
vel = xd_wb(19:21, :);
eulrate = rpy2eul(xd_wb(22:24,:));
qJd = xd_wb(25:end,:);

w = zeros(size(eulrate));
for i = 1:size(w, 2)
    w(:,i) = eulRate2angRate(eul(:,i))*eulrate(:,i);
end

Fd = [0,0,15]'; 
w = zeros(size(w));
xd_kd = [eul; pos; w; vel; qJ];
kdref.xd = num2cell(xd_kd,1);
kdref.ud = repmat({[Fd; Fd; Fd; Fd; zeros(12, 1)]}, 1, len_horizon-1);
end