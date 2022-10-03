function visual_kd_ddp_wrapper(hybridT, ou_iter, in_iter)
% hybridT -> 1xn array of hybrid trajectoris
X = [];
U = [];
for i = 1:length(hybridT)
    X = [X, cell2mat(hybridT(i).Xbar)];
    U = [U, cell2mat(hybridT(i).Ubar), zeros(24,1)];
end
len_horizons = [hybridT(:).len];
ctactSeq = {hybridT(:).ctact};
ctacts = [];
for i = 1:length(len_horizons)
    ctacts = [ctacts, repmat(ctactSeq{i}', 1, len_horizons(i))];
end

eul = X(1:3, :);
pos = X(4:6, :);
qdummy = X(13:24, :);
GRF = U(1:12,:);



graphic_option.show_footloc = true;
graphic_option.mode = "hkd";
graphic_option.show_floor = true;
graphic_option.show_GRF = false;
graphic_option.hide_leg = false;

Ni = 2;
graphics_data.eul = eul;
graphics_data.pos = pos;
graphics_data.qdummy = qdummy;
graphics_data.F = GRF;
graphics_data.ctacts = ctacts;
graphics_data.time = 1:Ni:size(GRF, 2);

%%
titletxt = sprintf('out iter = %d, in iter = %d', ou_iter, in_iter);
visualizeMCTrajectory(graphics_data, graphic_option,titletxt);

end