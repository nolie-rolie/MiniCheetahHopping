function hideBall(o)
if isfield(o, 'objs')
    o = o.objs;
end
set(o, 'FaceAlpha', 0);
end