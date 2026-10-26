function map = prepare_static_context(map,vehicle)
%PREPARE_STATIC_CONTEXT 缓存一次车辆矩形和原始静态几何，避免每个采样点重新校验。
% 输入：summon_map地图、车辆尺寸/裕度；输出：附加collision_context的地图副本。
% 矩形道路缓存精确边界；其他凸/凹多边形仍执行原完整几何判定。
margin = struct('front',0,'rear',0,'side',0);
if isfield(vehicle,'collision_margin_m'), margin = vehicle.collision_margin_m; end
% 可重复调用：源尺寸、裕度或任何静态几何改变后必须重新验证并缓存。
if isfield(map,'collision_context') && ...
        isequal(map.collision_context.dimensions_m,vehicle.dimensions_m) && ...
        isequal(map.collision_context.margin,margin) && ...
        isequal(map.collision_context.source_boundary,map.boundary_xy) && ...
        isequal(map.collision_context.source_obstacles,map.obstacles)
    return;
end
footprint = inflate_vehicle_occupancy(vehicle,0);
boundary = vertices(map.boundary_xy);
edges = boundary([2:end,1],:)-boundary;
next_edges = edges([2:end,1],:);
turns = edges(:,1).*next_edges(:,2)-edges(:,2).*next_edges(:,1);
turns = turns(abs(turns)>1e-12);
convex = isempty(turns) || all(turns>0) || all(turns<0);
bounds = [min(boundary,[],1),max(boundary,[],1)];
rectangle = size(boundary,1)==4 && convex && ...
    all(ismember(boundary(:,1),bounds([1,3]))) && ...
    all(ismember(boundary(:,2),bounds([2,4]))) && size(unique(boundary,'rows'),1)==4;
obstacles = cell(size(map.obstacles)); obstacle_bounds = zeros(numel(obstacles),4);
for k = 1:numel(obstacles)
    obstacles{k} = vertices(map.obstacles{k});
    obstacle_bounds(k,:) = [min(obstacles{k},[],1),max(obstacles{k},[],1)];
end
map.collision_context = struct('dimensions_m',vehicle.dimensions_m,'margin',margin, ...
    'source_boundary',map.boundary_xy,'source_obstacles',{map.obstacles}, ...
    'footprint',footprint,'boundary',boundary,'boundary_is_convex',convex, ...
    'is_rectangle',rectangle,'bounds',bounds,'obstacles',{obstacles},'obstacle_bounds',obstacle_bounds);
end

function polygon = vertices(polygon)
%VERTICES 只在建立上下文时验证静态多边形并去除重复末点。
polygon = double(polygon);
if size(polygon,2)~=2 || size(polygon,1)<3 || any(~isfinite(polygon(:)))
    error('Day6:StaticCollision:InvalidPolygon','静态几何须为有限Nx2多边形。');
end
if isequal(polygon(1,:),polygon(end,:)), polygon(end,:) = []; end
end
