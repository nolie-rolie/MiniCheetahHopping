clc
clear
%% process high-level data
process_option.filter_short_contact = false;
process_option.refine_trajectory = true;
process_option.dt = 0.003;
[hybridRef_WB, ctactSeq_all] = process_high_level_data(process_option); % ctactSeq has one more element than hybridRef
hybridRef_all = cellfun(@wbref2kdef, hybridRef_WB, 'UniformOutput',false);
dt = process_option.dt;

hybridRef_all = hybridRef_all(1:30);
ctactSeq_all = ctactSeq_all(1:30);

%% visulization
Xsim_cell = [];
for i = 1:length(hybridRef_all)
    Xsim_cell{end+1} = hybridRef_all{i}.xd;
end
Xsim = cell2mat([Xsim_cell{:}]);
eul = Xsim(1:3, :);
pos = Xsim(4:6, :);
qJ = Xsim(13:24, :);

graphic_option.show_footloc = false;
graphic_option.mode = "hkd";
graphic_option.show_floor = true;
graphic_option.show_GRF = false;
graphic_option.hide_leg = false;

Ni = 10;
graphics_data.eul = eul;
graphics_data.pos = pos;
graphics_data.qJ = qJ;
graphics_data.pf = [];
graphics_data.F = [];
graphics_data.ctacts = [];
graphics_data.time = 1:Ni:size(eul, 2);

visualizeMCTrajectory(graphics_data, graphic_option);