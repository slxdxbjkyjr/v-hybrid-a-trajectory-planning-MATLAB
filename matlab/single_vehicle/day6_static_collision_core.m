function [is_collision, detail] = day6_static_collision_core(pose,context)
%DAY6_STATIC_COLLISION_CORE 已准备且冻结的静态几何上下文专用检测内核。
% 输入：已经验证的1x3 double位姿、prepare_static_context产生的context。
% 调用期间不得修改地图或车辆配置；外部可变输入须用公共检查入口。
% 输出与day6_static_collision_check一致；仅请求布尔值时不创建detail。
footprint = context.footprint;
c = cos(pose(3)); s = sin(pose(3));
corners = footprint.corners_local*[c,s;-s,c]+pose(1:2);
boundary = context.boundary;
collect_detail = nargout > 1;
if collect_detail
    detail = struct('type','none','obstacle_index',0,'corners_xy',corners);
end
if context.is_rectangle
    % 道路角点贴边允许；保留原算式及严格不等号的数值边界语义。
    b = context.bounds;
    is_collision = any(corners(:,1)<b(1) | corners(:,1)>b(3) | ...
        corners(:,2)<b(2) | corners(:,2)>b(4));
else
    is_collision = ~all(inpolygon(corners(:,1),corners(:,2),boundary(:,1),boundary(:,2)));
end
if ~is_collision && ~context.boundary_is_convex
    % 凸边界中四角在内即保证矩形在内；凹边界必须检查车身边穿界。
    is_collision = boundaryCrossed(corners,boundary);
end
if is_collision
    if collect_detail, detail.type = 'road_boundary'; end
    return;
end
for k = 1:numel(context.obstacles)
    obstacle = context.obstacles{k};
    % 包围盒仅用于快速排除明显分离，多边形判定仍是最后判据。
    ob = context.obstacle_bounds(k,:);
    if max(corners(:,1)) < ob(1) || ob(3) < min(corners(:,1)) || ...
            max(corners(:,2)) < ob(2) || ob(4) < min(corners(:,2))
        continue;
    end
    overlap = any(inpolygon(corners(:,1),corners(:,2),obstacle(:,1),obstacle(:,2))) || ...
        any(inpolygon(obstacle(:,1),obstacle(:,2),corners(:,1),corners(:,2)));
    if ~overlap
        for i = 1:4
            a = corners(i,:); b = corners(mod(i,4)+1,:);
            for j = 1:size(obstacle,1)
                p = obstacle(j,:); q = obstacle(mod(j,size(obstacle,1))+1,:);
                if ~isempty(segmentParameters(a,b,p,q))
                    overlap = true; break;
                end
            end
            if overlap, break; end
        end
    end
    if overlap
        is_collision = true;
        if collect_detail
            detail.type = 'static_obstacle'; detail.obstacle_index = k;
        end
        return;
    end
end
end

function crossed = boundaryCrossed(corners,boundary)
%BOUNDARYCROSSED 按交点切分车身边，任何边段位于凹边界外即为越界。
crossed = false;
for i = 1:4
    a = corners(i,:); b = corners(mod(i,4)+1,:); parameters = [0,1];
    for j = 1:size(boundary,1)
        parameters = [parameters,segmentParameters(a,b,boundary(j,:), ...
            boundary(mod(j,size(boundary,1))+1,:))]; %#ok<AGROW>
    end
    parameters = unique(min(1,max(0,parameters)));
    mids = (parameters(1:end-1)+parameters(2:end))/2;
    probes = a+mids(:)*(b-a);
    if any(~inpolygon(probes(:,1),probes(:,2),boundary(:,1),boundary(:,2)))
        crossed = true; return;
    end
end
end

function parameters = segmentParameters(a,b,p,q)
%SEGMENTPARAMETERS 求线段ab与pq交点在ab上的参数，保留接触和共线重叠。
r = b-a; u = q-p; offset = p-a; denominator = cross2(r,u);
tol = 1e-12*max(1,norm(r)*norm(u)); parameters = [];
if abs(denominator)>tol
    t = cross2(offset,u)/denominator; v = cross2(offset,r)/denominator;
    if t>=-1e-12 && t<=1+1e-12 && v>=-1e-12 && v<=1+1e-12
        parameters = min(1,max(0,t));
    end
elseif abs(cross2(offset,r))<=tol
    length2 = dot(r,r);
    if length2<=eps, return; end
    interval = sort([dot(p-a,r),dot(q-a,r)]/length2);
    low = max(0,interval(1)); high = min(1,interval(2));
    if low<=high+1e-12, parameters = [low,high]; end
end
end

function value = cross2(a,b)
%CROSS2 二维向量叉积标量，用于方向判断和线段交点求解。
value = a(1)*b(2)-a(2)*b(1);
end
