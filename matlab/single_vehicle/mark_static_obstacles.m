function [map, statistics] = mark_static_obstacles(map, geometry, vehicle_config)
%MARK_STATIC_OBSTACLES 永久标记边界和按车辆尺寸膨胀的静态障碍物。
% 输入：时空地图、summon_map的输出、车辆配置。输出：更新地图与二维块统计。
% 无yaw维的静态图采用后轴膨胀矩形全航向包络，保守半径来自最远车角。
% 加半块对角线保证整个资源块可行；支持凹边界/多边形长边，不只查顶点。
if ~isstruct(geometry) || ~isfield(geometry,'boundary_xy')
    error('STOccupancy:InvalidGeometry','静态地图需要boundary_xy。');
end
boundary = checkedPolygon(geometry.boundary_xy);
footprint = inflate_vehicle_occupancy(vehicle_config,map.config.safety_distance_m);
radius = footprint.radius+hypot(map.dx,map.dy)/2;
[X,Y] = ndgrid(map.x_coords,map.y_coords);
inside = inpolygon(X,Y,boundary(:,1),boundary(:,2));
near_boundary = st_polyline_distance(X,Y,boundary) <= radius;
boundary_mask = ~inside | near_boundary;
map = st_write_occupancy(map,allLayers(find(boundary_mask),map),'parking_lot_boundary',1);
statistics = struct('boundary_xy_blocks',nnz(boundary_mask),'static_xy_blocks',0, ...
    'inflation_radius_m',footprint.radius,'cell_padding_m',hypot(map.dx,map.dy)/2);
if isfield(geometry,'obstacles')
    for k = 1:numel(geometry.obstacles)
        polygon = checkedPolygon(geometry.obstacles{k});
        hit = inpolygon(X,Y,polygon(:,1),polygon(:,2)) | ...
            st_polyline_distance(X,Y,polygon) <= radius;
        owner_id = sprintf('static_obstacle_%03d',k);
        if isfield(geometry,'obstacle_ids'), owner_id = geometry.obstacle_ids{k}; end
        map = st_write_occupancy(map,allLayers(find(hit),map),owner_id,2);
        statistics.static_xy_blocks = statistics.static_xy_blocks+nnz(hit);
    end
end
map.geometry = geometry;
map.static_statistics = statistics;
end

function polygon = checkedPolygon(polygon)
%CHECKEDPOLYGON 输入为有限N×2简单多边形，面积必须非零。
if ~isnumeric(polygon) || ~isreal(polygon) || size(polygon,2) ~= 2 || ...
        size(polygon,1) < 3 || any(~isfinite(polygon(:))) || polyarea(polygon(:,1),polygon(:,2)) <= 0
    error('STOccupancy:InvalidGeometry','边界和障碍物必须为非退化有限N×2多边形。');
end
polygon = double(polygon);
end

function indices = allLayers(xy_indices,map)
%ALLLAYERS 将二维块沿时间轴复制到每一层。
indices = xy_indices(:)+(0:map.nt-1)*(map.nx*map.ny);
indices = indices(:);
end
