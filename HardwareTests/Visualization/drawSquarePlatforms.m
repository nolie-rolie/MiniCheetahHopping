function drawSquarePlatforms(platforms)

for i = 1:length(platforms(:,1))
    p1 = [platforms(i,1) platforms(i,3) platforms(i,5)];
    p2 = [platforms(i,2) platforms(i,3) platforms(i,5)];
    p3 = [platforms(i,2) platforms(i,4) platforms(i,5)];
    p4 = [platforms(i,1) platforms(i,4) platforms(i,5)]; 

    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    
    if mod(i,2)==0
        fill3(x, y, z, [174, 143, 64]/256);
    else
        fill3(x, y, z, [10, 134, 61]/256);
    end
    hold on
end

end