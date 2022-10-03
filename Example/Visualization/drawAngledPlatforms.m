function drawAngledPlatforms(platforms)

for i = 1:length(platforms(:,1))
    [x,y] = meshgrid(platforms(i,1):0.1:platforms(i,2),platforms(i,3):.1:platforms(i,4));
    if platforms(i,7) == 0
        z = platforms(i,5) - platforms(i,6)/platforms(i,8)*(x - (platforms(i,2)-platforms(i,1)));
    end
    if platforms(i,6) == 0
        z = platforms(i,5) - platforms(i,7)/platforms(i,8)*(y - (platforms(i,4)-platforms(i,3)));
    end
    s1 = surf(x,y,z);
    set(s1,'FaceColor',[12, 35, 64]/256)
    hold on
end

end