function report = vhybrid_validate_path_geometry(candidate, map, vehicle_config, planner_config, optimization_config)
%vhybrid_validate_path_geometry - Validate a geometric path candidate
%   REPORT = vhybrid_validate_path_geometry(CANDIDATE,MAP,VEHICLE_CONFIG,
%   PLANNER_CONFIG,OPTIMIZATION_CONFIG) independently checks fixed anchors,
%   swept static vehicle rectangles and the project maximum steering angle.
%   Curvature uses the course direction and the candidate gear sign.
%
%   This function does not validate steering rate, time parameterization or
%   multi-vehicle space-time collision. A passing candidate is not executable.
%
%   See also vhybrid_optimize_path, day6_static_collision_check

if nargin < 5
    error('VHybridPath:MissingInput','Five inputs are required.');
end
cfg = validateConfig(optimization_config);
required = {'x','y','theta','gear','run_id','anchor_mask', ...
    'reference_xy','reference_theta'};
if ~isstruct(candidate) || ~isscalar(candidate) || ...
        ~all(isfield(candidate,required))
    report = invalidReport('missing_candidate_geometry_fields');
    return;
end
n = numel(candidate.x);
fields = {'y','theta','gear','run_id','anchor_mask','reference_theta'};
if n<2 || any(cellfun(@(name)numel(candidate.(name))~=n,fields)) || ...
        ~isequal(size(candidate.reference_xy),[n,2])
    report = invalidReport('invalid_candidate_shape');
    return;
end
values = [double(candidate.x(:)),double(candidate.y(:)), ...
    double(candidate.theta(:)),double(candidate.gear(:)),double(candidate.run_id(:)), ...
    double(candidate.reference_xy),double(candidate.reference_theta(:))];
if any(~isfinite(values(:))) || any(~ismember(candidate.gear(:),[-1,1])) || ...
        ~islogical(candidate.anchor_mask) || numel(candidate.anchor_mask)~=n
    report = invalidReport('nonfinite_or_invalid_candidate_geometry');
    return;
end
if isfield(candidate,'is_executable') && candidate.is_executable
    report = invalidReport('geometric_candidate_must_not_be_executable');
    return;
end

wheelbase = vehicle_config.dimensions_m.wheelbase;
delta_max = min(vehicle_config.steering.maximum_steer_rad, ...
    planner_config.dynamics.delta_max_rad);
report = struct('valid',false,'status','failed','reason','not_checked', ...
    'static_collision_free',false,'steering_within_limit',false, ...
    'max_steer_angle_rad',0,'max_abs_curvature_per_m',0, ...
    'maximum_steering_angle_rad',delta_max,'steering_tolerance_rad', ...
    cfg.validation.steering_tolerance_rad,'swept_pose_count',0, ...
    'swept_segment_count',0,'sweep_method','fixed_step_rectangle_pose_sampling', ...
    'sweep_step_m',cfg.validation.static_sweep_step_m, ...
    'sweep_heading_step_rad',cfg.validation.static_sweep_heading_step_rad, ...
    'collision_count',0,'first_collision',struct(), ...
    'anchor_position_error_m',0,'anchor_heading_error_rad',0, ...
    'turning_rate_status','not_assessed_day9', ...
    'spatiotemporal_collision_status','not_assessed_day9', ...
    'failure_reasons',{{}});

anchor = logical(candidate.anchor_mask(:));
if any(anchor)
    report.anchor_position_error_m = max(hypot( ...
        double(candidate.x(anchor))-candidate.reference_xy(anchor,1), ...
        double(candidate.y(anchor))-candidate.reference_xy(anchor,2)));
    report.anchor_heading_error_rad = max(abs(wrapAngle( ...
        double(candidate.theta(anchor))-double(candidate.reference_theta(anchor)))));
end
anchor_tol = cfg.anchor_position_tolerance_m;
if report.anchor_position_error_m > anchor_tol || ...
        report.anchor_heading_error_rad > cfg.anchor_heading_tolerance_rad
    report.failure_reasons{end+1} = 'anchor_state_changed';
end

map = prepare_static_context(map,vehicle_config);
static_context = map.collision_context;
first_collision = struct('point_index',NaN,'segment_index',NaN, ...
    'sample_index',NaN,'position_xy',[NaN,NaN],'heading_rad',NaN,'type','');
collision_count = 0;
pose_count = 0;
for k = 1:n
    [hit,detail] = day6_static_collision_core( ...
        [candidate.x(k),candidate.y(k),candidate.theta(k)],static_context);
    pose_count = pose_count+1;
    if hit
        collision_count = collision_count+1;
        if isnan(first_collision.point_index)
            first_collision.point_index = k;
            first_collision.position_xy = [candidate.x(k),candidate.y(k)];
            first_collision.heading_rad = candidate.theta(k);
            first_collision.type = detail.type;
        end
    end
end
for k = 1:n-1
    dx = candidate.x(k+1)-candidate.x(k);
    dy = candidate.y(k+1)-candidate.y(k);
    distance = hypot(dx,dy);
    yaw_change = abs(wrapAngle(candidate.theta(k+1)-candidate.theta(k)));
    if distance<=eps && yaw_change<=eps
        continue;
    end
    report.swept_segment_count = report.swept_segment_count+1;
    sample_count = max([1,ceil(distance/cfg.validation.static_sweep_step_m), ...
        ceil(yaw_change/cfg.validation.static_sweep_heading_step_rad)]);
    for sample = 1:sample_count-1
        ratio = sample/sample_count;
        position = [candidate.x(k),candidate.y(k)]+ratio*[dx,dy];
        yaw = candidate.theta(k)+ratio*wrapAngle(candidate.theta(k+1)-candidate.theta(k));
        [hit,detail] = day6_static_collision_core([position,yaw],static_context);
        pose_count = pose_count+1;
        if hit
            collision_count = collision_count+1;
            if isnan(first_collision.point_index)
                first_collision.segment_index = k;
                first_collision.sample_index = sample;
                first_collision.position_xy = position;
                first_collision.heading_rad = yaw;
                first_collision.type = detail.type;
            end
        end
    end
end

curvature = zeros(n,1);
for run_id = unique(candidate.run_id(candidate.run_id>0)).'
    idx = find(candidate.run_id==run_id);
    if numel(idx)<2, continue; end
    edge = diff([candidate.x(idx),candidate.y(idx)],1,1);
    lengths = hypot(edge(:,1),edge(:,2));
    if any(lengths<=eps)
        report.failure_reasons{end+1} = 'zero_length_edge_inside_motion_run';
        continue;
    end
    course_yaw = unwrap(atan2(edge(:,2),edge(:,1)));
    if numel(idx)>=3
        local = diff(course_yaw)./(0.5*(lengths(1:end-1)+lengths(2:end)));
        body_curvature = candidate.gear(idx(1))*local;
        curvature(idx(2:end-1)) = body_curvature;
        curvature(idx(1)) = body_curvature(1);
        curvature(idx(end)) = body_curvature(end);
    end
end
steer = atan(wheelbase*curvature);
report.max_abs_curvature_per_m = max(abs(curvature));
report.max_steer_angle_rad = max(abs(steer));
report.steering_within_limit = report.max_steer_angle_rad <= ...
    delta_max+cfg.validation.steering_tolerance_rad;
report.static_collision_free = collision_count==0;
report.collision_count = collision_count;
report.swept_pose_count = pose_count;
report.first_collision = first_collision;
if ~report.static_collision_free
    report.failure_reasons{end+1} = 'static_rectangle_sweep_collision';
end
if ~report.steering_within_limit
    report.failure_reasons{end+1} = 'maximum_steering_angle_exceeded';
end
report.failure_reasons = unique(report.failure_reasons,'stable');
report.valid = isempty(report.failure_reasons);
report.status = conditional(report.valid,'passed','failed');
report.reason = conditional(report.valid,'passed',strjoin(report.failure_reasons,';'));
end

function cfg = validateConfig(input)
%VALIDATECONFIG Require explicit geometric validation sampling and tolerances.
required = {'validation','anchor_position_tolerance_m','anchor_heading_tolerance_rad'};
if ~isstruct(input) || ~isscalar(input) || ~all(isfield(input,required)) || ...
        ~isstruct(input.validation) || ~isscalar(input.validation)
    error('VHybridPath:InvalidConfig','Geometric validation configuration is incomplete.');
end
names = {'static_sweep_step_m','static_sweep_heading_step_rad','steering_tolerance_rad'};
for k = 1:numel(names)
    value = input.validation.(names{k});
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value<=0
        error('VHybridPath:InvalidConfig','Validation step and tolerance must be positive.');
    end
end
cfg = input;
end

function report = invalidReport(reason)
%INVALIDREPORT Return a failed report before attempting geometric operations.
report = struct('valid',false,'status','failed','reason',reason, ...
    'static_collision_free',false,'steering_within_limit',false, ...
    'max_steer_angle_rad',NaN,'max_abs_curvature_per_m',NaN, ...
    'maximum_steering_angle_rad',NaN,'steering_tolerance_rad',NaN, ...
    'swept_pose_count',0,'swept_segment_count',0, ...
    'sweep_method','fixed_step_rectangle_pose_sampling', ...
    'sweep_step_m',NaN,'sweep_heading_step_rad',NaN, ...
    'collision_count',0,'first_collision',struct(), ...
    'anchor_position_error_m',NaN,'anchor_heading_error_rad',NaN, ...
    'turning_rate_status','not_assessed_day9', ...
    'spatiotemporal_collision_status','not_assessed_day9', ...
    'failure_reasons',{{reason}});
end

function angle = wrapAngle(angle)
%WRAPANGLE Return signed angle differences in [-pi,pi).
angle = mod(angle+pi,2*pi)-pi;
end

function value = conditional(flag,a,b)
%CONDITIONAL Select a status string without evaluating dynamic geometry.
if flag, value = a; else, value = b; end
end
