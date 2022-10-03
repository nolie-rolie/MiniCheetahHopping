function o = updateBall(o, p, C, alpha)
c = o.c;
set(o.objs, 'XData', o.objs.XData + p(1)-c(1));
set(o.objs, 'YData', o.objs.YData + p(2)-c(2));
set(o.objs, 'ZData', o.objs.ZData + p(3)-c(3));
set(o.objs, 'CData', C);
set(o.objs, 'FaceAlpha', alpha);
set(o.objs, 'Visible', 'on');
o.c = p;
end