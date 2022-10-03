function drawObstacles(obstacles)

for i = 1:length(obstacles(:,1))
    x_wall = 0;
    if obstacles(i,1) == obstacles(i,2)
        x_wall = 1;
    end
    if x_wall == 1
        [y,z] = meshgrid(obstacles(i,3):0.1:obstacles(i,4),obstacles(i,5):0.1:obstacles(i,6));
        x = ones(size(y))*obstacles(i,1);
    else
        [x,z] = meshgrid(obstacles(i,1):0.1:obstacles(i,2),obstacles(i,5):0.1:obstacles(i,6));
        y = ones(size(x))*obstacles(i,3);
    end
    wall = surf(x,y,z);
    set(wall,'FaceColor',[12, 35, 64]/256);
    hold on
end

end