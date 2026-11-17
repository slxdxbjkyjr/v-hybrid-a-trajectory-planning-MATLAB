function [corridor, report] = vhybrid_build_path_corridor(reference_path, map, vehicle_config, optimization_config)
%vhybrid_build_path_corridor - Build local vehicle-safe waypoint boxes
%   [CORRIDOR,REPORT] = vhybrid_build_path_corridor(REFERENCE_PATH,MAP,
%   VEHICLE_CONFIG,OPTIMIZATION_CONFIG) builds convex local boxes around
%   an arc-length-resampled reference path. Inputs use the project map and
%   vehicle structures. Each free box is bounded by the exact reference
%   vehicle-to-obstacle/boundary clearance, a rotation allowance and the
%   configured width. Anchors remain fixed.
%
%   CORRIDOR contains lower/upper N-by-2 arrays and per-point clearance.
%   REPORT.valid is false if any reference pose is unsafe or any free box
%   is narrower than the configured minimum.
%
%   See also vhybrid_optimize_path, vhybrid_validate_path_geometry

if nargin < 4
    error('VHybridPath:MissingInput','Four inputs are required.');
end
cfg = validateConfig(optimization_config);
required = {'x','y','theta','gear','run_id','anchor_mask'};
if ~isstruct(reference_path) || ~isscalar(reference_path) || ...
        ~all(isfield(reference_path,required))
    error('VHybridPath:InvalidReference','Reference path is missing geometry metadata.');
end
n = numel(reference_path.x);
if n < 2 || any([numel(reference_path.y),numel(reference_path.theta), ...
        numel(reference_path.gear),numel(reference_path.run_id), ...
        numel(reference_path.anchor_mask)] ~= n)
    error('VHybridPath:InvalidReference','Reference path fields must have equal length >= 2.');
end

template = struct('reference_xy',[NaN,NaN],'clearance_m',NaN, ...
    'rotation_allowance_m',NaN,'mu_index',NaN,'paper_half_width_m',NaN, ...
    'safe_half_width_m',NaN,'epsilon_xy_m',[NaN,NaN], ...
    'anchor',false,'valid',false,'reason','not_checked');
points = repmat(template,n,1);
corridor = struct('lower',nan(n,2),'upper',nan(n,2), ...
    'half_width',nan(n,1),'points',points, ...
    'construction_method',cfg.corridor.construction_method);
report = struct('valid',false,'status','failed','reason','not_checked', ...
    'point_count',n,'failed_point_index',NaN,'points',points, ...
    'footprint_model','exact project rectangle with configured margins', ...
    'heading_uncertainty_rad',cfg.corridor.heading_uncertainty_rad);
if ~strcmp(cfg.corridor.construction_method,'vehicle_clearance_local_axis_aligned_box')
    report.reason = 'unsupported_corridor_construction_method';
    return;
end

map = prepare_static_context(map,vehicle_config);
footprint = inflate_vehicle_occupancy(vehicle_config,0);
for k = 1:n
    pose = [double(reference_path.x(k)),double(reference_path.y(k)), ...
        double(reference_path.theta(k))];
    point = template;
    point.reference_xy = pose(1:2);
    point.anchor = logical(reference_path.anchor_mask(k));
    [hit,detail] = day6_static_collision_check(pose,map,vehicle_config);
    if hit
        point.reason = 'reference_vehicle_collision';
        points(k) = point;
        report.failed_point_index = k;
        report.reason = point.reason;
        report.points = points;
        corridor.points = points;
        return;
    end

    clearance = polygonGap(detail.corners_xy,map.boundary_xy);
    for j = 1:numel(map.obstacles)
        clearance = min(clearance,polygonGap(detail.corners_xy,map.obstacles{j}));
    end
    rotation_allowance = 2*footprint.radius* ...
        sin(min(pi,cfg.corridor.heading_uncertainty_rad)/2);
    available = clearance-rotation_allowance-cfg.corridor.clearance_reserve_m;
    if ~isfinite(clearance) || available <= 0
        point.clearance_m = clearance;
        point.rotation_allowance_m = rotation_allowance;
        point.reason = 'insufficient_vehicle_clearance';
        points(k) = point;
        report.failed_point_index = k;
        report.reason = point.reason;
        report.points = points;
        corridor.points = points;
        return;
    end

    mu = distanceToRunEndpoint(k,reference_path.run_id);
    taper = 1;
    if cfg.corridor.paper_index_taper_enabled && reference_path.run_id(k) > 0
        taper = 1/(1+exp(4-mu));
    end
    paper_half = cfg.corridor.base_clearance_width_m/sqrt(2)*taper;
    safe_half = min(paper_half,available/sqrt(2));
    safe_half = min(safe_half,cfg.corridor.base_clearance_width_m/sqrt(2));
    if point.anchor
        safe_half = 0;
    elseif safe_half < cfg.corridor.minimum_half_width_m
        point.clearance_m = clearance;
        point.rotation_allowance_m = rotation_allowance;
        point.mu_index = mu;
        point.paper_half_width_m = paper_half;
        point.safe_half_width_m = safe_half;
        point.reason = 'corridor_narrower_than_minimum';
        points(k) = point;
        report.failed_point_index = k;
        report.reason = point.reason;
        report.points = points;
        corridor.points = points;
        return;
    end

    epsilon = max(0,paper_half-safe_half);
    lower = pose(1:2)-paper_half+epsilon;
    upper = pose(1:2)+paper_half-epsilon;
    if point.anchor
        lower = pose(1:2);
        upper = pose(1:2);
    end
    point.clearance_m = clearance;
    point.rotation_allowance_m = rotation_allowance;
    point.mu_index = mu;
    point.paper_half_width_m = paper_half;
    point.safe_half_width_m = safe_half;
    point.epsilon_xy_m = [epsilon,epsilon];
    point.valid = true;
    point.reason = 'passed';
    corridor.lower(k,:) = lower;
    corridor.upper(k,:) = upper;
    corridor.half_width(k) = safe_half;
    points(k) = point;
end
corridor.points = points;
report.points = points;
report.valid = true;
report.status = 'passed';
report.reason = 'passed';
end

function cfg = validateConfig(input)
%VALIDATECONFIG Check required corridor values before geometric work.
if ~isstruct(input) || ~isscalar(input) || ~isfield(input,'corridor') || ...
        ~isstruct(input.corridor) || ~isscalar(input.corridor)
    error('VHybridPath:InvalidConfig','Configuration needs a corridor section.');
end
cfg = input;
required = {'construction_method','base_clearance_width_m', ...
    'minimum_half_width_m','clearance_reserve_m','heading_uncertainty_rad', ...
    'paper_index_taper_enabled','minimum_neighbor_projection_m'};
if ~all(isfield(cfg.corridor,required))
    error('VHybridPath:InvalidConfig','Corridor configuration is incomplete.');
end
numeric = {'base_clearance_width_m','minimum_half_width_m', ...
    'clearance_reserve_m','heading_uncertainty_rad', ...
    'minimum_neighbor_projection_m'};
for k = 1:numel(numeric)
    value = cfg.corridor.(numeric{k});
    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value)
        error('VHybridPath:InvalidConfig','Corridor values must be finite scalars.');
    end
end
if cfg.corridor.base_clearance_width_m <= 0 || ...
        cfg.corridor.minimum_half_width_m <= 0 || ...
        cfg.corridor.clearance_reserve_m < 0 || ...
        cfg.corridor.heading_uncertainty_rad < 0 || ...
        cfg.corridor.heading_uncertainty_rad > pi || ...
        cfg.corridor.minimum_neighbor_projection_m <= 0 || ...
        ~islogical(cfg.corridor.paper_index_taper_enabled) || ...
        ~isscalar(cfg.corridor.paper_index_taper_enabled)
    error('VHybridPath:InvalidConfig','Corridor widths and heading bound are invalid.');
end
end

function mu = distanceToRunEndpoint(index,run_ids)
%DISTANCETORUNENDPOINT Use the paper's unnormalized waypoint-index distance.
run_id = run_ids(index);
if run_id <= 0
    mu = 0;
    return;
end
indices = find(run_ids==run_id);
local_index = find(indices==index,1);
mu = min(local_index-1,numel(indices)-local_index);
end

function distance = polygonGap(a,b)
%POLYGONGAP Minimum edge-to-edge distance between disjoint polygons.
a = double(a); b = double(b);
distance = Inf;
for i = 1:size(a,1)
    a1 = a(i,:); a2 = a(mod(i,size(a,1))+1,:);
    for j = 1:size(b,1)
        b1 = b(j,:); b2 = b(mod(j,size(b,1))+1,:);
        if segmentsIntersect(a1,a2,b1,b2)
            distance = 0;
            return;
        end
        distance = min(distance,min([pointSegmentDistance(a1,b1,b2), ...
            pointSegmentDistance(a2,b1,b2),pointSegmentDistance(b1,a1,a2), ...
            pointSegmentDistance(b2,a1,a2)]));
    end
end
end

function distance = pointSegmentDistance(p,a,b)
%POINTSEGMENTDISTANCE Euclidean distance from a point to a closed segment.
v = b-a;
denominator = dot(v,v);
if denominator <= eps
    distance = norm(p-a);
    return;
end
t = min(1,max(0,dot(p-a,v)/denominator));
distance = norm(p-(a+t*v));
end

function hit = segmentsIntersect(a,b,c,d)
%SEGMENTSINTERSECT Include proper crossings, endpoint contact and collinearity.
tol = 1e-12;
ab = b-a; cd = d-c;
den = cross2(ab,cd);
offset = c-a;
if abs(den)>tol
    t = cross2(offset,cd)/den;
    u = cross2(offset,ab)/den;
    hit = t>=-tol && t<=1+tol && u>=-tol && u<=1+tol;
    return;
end
if abs(cross2(offset,ab))>tol
    hit = false;
    return;
end
length2 = dot(ab,ab);
if length2<=eps
    hit = norm(a-c)<=tol || norm(a-d)<=tol;
    return;
end
interval = sort([dot(c-a,ab),dot(d-a,ab)]/length2);
hit = max(0,interval(1))<=min(1,interval(2))+tol;
end

function value = cross2(a,b)
%CROSS2 Scalar cross product in two dimensions.
value = a(1)*b(2)-a(2)*b(1);
end
