function testDataProcess()
%process high-level data
process_option.filter_short_contact = false;
process_option.refine_trajectory = false;
[hybridRef_WB, ctactSeq] = process_high_level_data(process_option); % ctactSeq has one more element than hybridRef
hybridRef = cellfun(@wbref2kdef, hybridRef_WB, 'UniformOutput',false);

nSeq = length(hybridRef);
X_hybrid = [];
U_hybrid = [];
for i = 1:nSeq
    X_hybrid{end+1} = hybridRef{i}.xd;
end

%% visulization
Xvsl = cell2mat([X_hybrid{:}]);
eul = Xvsl(1:3, :);
pos = Xvsl(4:6, :);
qJ = Xvsl(13:24, :);

graphic_option.show_footloc = false;
graphic_option.mode = "hkd";
graphic_option.show_floor = true;
graphic_option.show_GRF = false;

Ni = 1;
graphics_data.eul = eul;
graphics_data.pos = pos;
graphics_data.qJ = qJ;
graphics_data.pf = [];
graphics_data.F = [];
graphics_data.ctacts = [];
graphics_data.time = 1:Ni:size(eul, 2);

visualizeMCTrajectory(graphics_data, graphic_option);
end