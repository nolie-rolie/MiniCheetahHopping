function o = drawBall(p, r, C, alpha)
[X, Y, Z] = sphere(); % unit sphere centered at origin
X = r*X + p(1);
Y = r*Y + p(2);
Z = r*Z + p(3);
obj = surf(X,Y,Z,'FaceColor',C, 'FaceAlpha',alpha,'EdgeAlpha',0);
o.objs = obj;
o.c = p; 
end