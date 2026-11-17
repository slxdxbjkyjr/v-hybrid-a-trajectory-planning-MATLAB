function [candidate, report] = vhybrid_optimize_path(rough_path, map, vehicle_config, planner_config, optimization_config)
%vhybrid_optimize_path - Optimize a geometric V-Hybrid A* path candidate
%   [CANDIDATE,REPORT] = vhybrid_optimize_path(ROUGH_PATH,MAP,
%   VEHICLE_CONFIG,PLANNER_CONFIG,OPTIMIZATION_CONFIG) resamples the rough
%   path by arc length, retains stops and gear-change anchors, solves a
%   corridor-constrained quadratic program, and validates the resulting
%   geometry with the project vehicle rectangle and steering limit.
%
%   CANDIDATE contains geometry only. It has no executable speed or time
%   profile, and never enters the Day 7 space-time occupancy workflow.
%   REPORT is failed explicitly when resampling, corridor construction,
%   optimization or independent geometry validation fails.
%
%   See also vhybrid_build_path_corridor, vhybrid_validate_path_geometry

if nargin < 5
    error('VHybridPath:MissingInput','Five inputs are required.');
end
cfg = validateConfig(optimization_config);
candidate = emptyCandidate();
report = struct('valid',false,'status','failed','reason','not_checked', ...
    'solver_name',cfg.solver.name,'solver_exitflag',NaN,'solver_output',struct(), ...
    'resampled_point_count',0,'optimized_run_count',0,'skipped_short_run_count',0, ...
    'smoothness_before',NaN,'smoothness_after',NaN, ...
    'spacing_error_before_m2',NaN,'spacing_error_after_m2',NaN, ...
    'corridor',struct(),'geometry_validation',struct(), ...
    'candidate_type','geometric_path_candidate_not_executable', ...
    'time_parameterized',false,'turning_rate_status','not_assessed_day9', ...
    'static_collision_free',false,'max_steer_angle_rad',NaN, ...
    'steering_within_limit',false, ...
    'spatiotemporal_collision_status','not_assessed_day9','failure_reasons',{{}});
if ~cfg.enabled
    report.reason = 'optimization_disabled';
    report.failure_reasons = {report.reason};
    return;
end
if ~isfield(planner_config,'dynamics') || ~isfield(vehicle_config,'dimensions_m')
    report.reason = 'missing_vehicle_or_planner_configuration';
    report.failure_reasons = {report.reason};
    return;
end
if ~isstruct(rough_path) || ~isscalar(rough_path) || ...
        ~all(isfield(rough_path,{'x','y','theta'})) || ...
        numel(rough_path.x)<2 || ...
        any([numel(rough_path.y),numel(rough_path.theta)]~=numel(rough_path.x))
    report.reason = 'invalid_rough_path';
    report.failure_reasons = {report.reason};
    return;
end

[candidate,resample_report] = resamplePath(rough_path,cfg);
if ~resample_report.valid
    report.reason = resample_report.reason;
    report.failure_reasons = {report.reason};
    return;
end
report.resampled_point_count = numel(candidate.x);
report.smoothness_before = pathSmoothness(candidate);
report.spacing_error_before_m2 = spacingError(candidate);

[corridor,corridor_report] = vhybrid_build_path_corridor( ...
    candidate,map,vehicle_config,cfg);
report.corridor = corridor_report;
if ~corridor_report.valid
    report.reason = ['corridor_' corridor_report.reason];
    report.failure_reasons = {report.reason};
    return;
end

run_ids = unique(candidate.run_id(candidate.run_id>0),'stable');
for r = run_ids(:).'
    indices = find(candidate.run_id==r);
    if numel(indices)<4
        report.skipped_short_run_count = report.skipped_short_run_count+1;
        continue;
    end
    [solution,solve_report] = solveRun(candidate,indices,corridor, ...
        vehicle_config,planner_config,cfg);
    report.solver_exitflag = solve_report.exitflag;
    report.solver_output = solve_report.output;
    if ~solve_report.success
        report.reason = solve_report.reason;
        report.failure_reasons = {report.reason};
        candidate.valid = false;
        return;
    end
    candidate.x(indices) = solution(1:numel(indices));
    candidate.y(indices) = solution(numel(indices)+1:end);
    report.optimized_run_count = report.optimized_run_count+1;
end

candidate = updateCandidateHeading(candidate);
report.smoothness_after = pathSmoothness(candidate);
report.spacing_error_after_m2 = spacingError(candidate);
geometry_report = vhybrid_validate_path_geometry( ...
    candidate,map,vehicle_config,planner_config,cfg);
report.geometry_validation = geometry_report;
report.static_collision_free = geometry_report.static_collision_free;
report.max_steer_angle_rad = geometry_report.max_steer_angle_rad;
report.steering_within_limit = geometry_report.steering_within_limit;
candidate.valid = geometry_report.valid;
candidate.is_executable = false;
candidate.time_parameterized = false;
report.valid = geometry_report.valid;
report.status = conditional(report.valid,'passed','failed');
report.reason = geometry_report.reason;
report.failure_reasons = geometry_report.failure_reasons;
if report.valid
    report.reason = 'passed_geometric_candidate_only';
end
candidate.validation = geometry_report;
candidate.optimization_status = report.status;
candidate.optimization_reason = report.reason;
end

function cfg = validateConfig(input)
%VALIDATECONFIG Validate the explicit solver, weights and sampling values.
required = {'enabled','resample_spacing_m','anchor_speed_tolerance_mps', ...
    'anchor_position_tolerance_m','anchor_heading_tolerance_rad', ...
    'weights','solver','validation','corridor'};
if ~isstruct(input) || ~isscalar(input) || ~all(isfield(input,required))
    error('VHybridPath:InvalidConfig','Path optimization configuration is incomplete.');
end
logicalFields = {'enabled'};
for k = 1:numel(logicalFields)
    value = input.(logicalFields{k});
    if ~islogical(value) || ~isscalar(value)
        error('VHybridPath:InvalidConfig','Optimization switches must be logical scalars.');
    end
end
numeric = {'resample_spacing_m','anchor_speed_tolerance_mps', ...
    'anchor_position_tolerance_m','anchor_heading_tolerance_rad'};
for k = 1:numel(numeric)
    value = input.(numeric{k});
    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value) || value<0
        error('VHybridPath:InvalidConfig','Spacing and tolerances must be finite nonnegative scalars.');
    end
end
if input.resample_spacing_m<=0 || input.anchor_position_tolerance_m<=0
    error('VHybridPath:InvalidConfig','Resampling spacing and anchor tolerance must be positive.');
end
if ~isstruct(input.weights) || ~isscalar(input.weights) || ...
        ~isstruct(input.solver) || ~isscalar(input.solver) || ...
        ~isstruct(input.corridor) || ~isscalar(input.corridor) || ...
        ~all(isfield(input.weights,{'smoothness','reference','spacing'})) || ...
        ~all(isfield(input.solver,{'name','algorithm','display','max_iterations', ...
        'constraint_tolerance','optimality_tolerance','step_tolerance'})) || ...
        ~all(isfield(input.weights,{'curvature_safety_factor', ...
        'curvature_linearization_iterations'}))
    error('VHybridPath:InvalidConfig','Weights or solver options are incomplete.');
end
weightNames = {'smoothness','reference','spacing'};
for k = 1:numel(weightNames)
    value = input.weights.(weightNames{k});
    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value) || value<0
        error('VHybridPath:InvalidConfig','Objective weights must be finite nonnegative scalars.');
    end
end
if input.weights.reference<=0 || sum([input.weights.smoothness, ...
        input.weights.reference,input.weights.spacing])<=0
    error('VHybridPath:InvalidConfig','Use a positive reference weight and nonzero objective.');
end
factor = input.weights.curvature_safety_factor;
iterations = input.weights.curvature_linearization_iterations;
if ~isnumeric(factor) || ~isscalar(factor) || ~isreal(factor) || ...
        ~isfinite(factor) || ~isnumeric(iterations) || ...
        ~isscalar(iterations) || ~isreal(iterations) || ~isfinite(iterations)
    error('VHybridPath:InvalidConfig','Curvature settings must be finite numeric scalars.');
end
if factor<=0 || ...
        factor>1 || iterations<1 || fix(iterations)~=iterations
    error('VHybridPath:InvalidConfig','Curvature settings are outside valid bounds.');
end
projection = input.corridor.minimum_neighbor_projection_m;
if ~isnumeric(projection) || ~isscalar(projection) || ~isreal(projection) || ...
        ~isfinite(projection) || projection<=0
    error('VHybridPath:InvalidConfig', ...
        'Minimum neighbor projection must be a finite positive scalar.');
end
cfg = input;
end

function candidate = emptyCandidate()
%EMPTYCANDIDATE Declare a geometry-only candidate with no dynamic fields.
candidate = struct('x',zeros(0,1),'y',zeros(0,1),'theta',zeros(0,1), ...
    'gear',zeros(0,1),'run_id',zeros(0,1),'anchor_mask',false(0,1), ...
    'source_index',zeros(0,1),'source_s_m',zeros(0,1), ...
    'reference_xy',zeros(0,2),'reference_theta',zeros(0,1), ...
    'anchor_reason',{{}},'candidate_type','geometric_path_candidate_not_executable', ...
    'is_executable',false,'time_parameterized',false,'valid',false);
end

function [candidate,report] = resamplePath(path,cfg)
%RESAMPLEPATH Preserve dwell/gear edges and resample each moving gear run.
n = numel(path.x);
x = double(path.x(:)); y = double(path.y(:)); theta = double(path.theta(:));
if any(~isfinite([x;y;theta]))
    candidate = emptyCandidate();
    report = struct('valid',false,'reason','nonfinite_rough_path');
    return;
end
gear = ones(n,1);
if isfield(path,'gear'), gear = double(path.gear(:)); end
if numel(gear)~=n || any(~ismember(gear,[-1,1]))
    candidate = emptyCandidate();
    report = struct('valid',false,'reason','invalid_rough_path_gear');
    return;
end
speed = ones(n,1);
if isfield(path,'v'), speed = abs(double(path.v(:))); end
if numel(speed)~=n || any(~isfinite(speed))
    candidate = emptyCandidate();
    report = struct('valid',false,'reason','invalid_rough_path_speed');
    return;
end
shift = false(n,1);
if isfield(path,'is_gear_change'), shift = logical(path.is_gear_change(:)); end
if numel(shift)~=n
    candidate = emptyCandidate();
    report = struct('valid',false,'reason','invalid_gear_change_flags');
    return;
end
distance = hypot(diff(x),diff(y));
source_s = [0;cumsum(distance)];
source_anchor = false(n,1);
source_anchor([1,n]) = true;
source_anchor(speed<=cfg.anchor_speed_tolerance_mps) = true;
for k = 2:n
    if shift(k) || gear(k)~=gear(k-1)
        source_anchor(k-1:k) = true;
    end
end
moving_edge = false(n,1);
for k = 2:n
    moving_edge(k) = distance(k-1)>cfg.anchor_position_tolerance_m && ...
        gear(k)==gear(k-1) && ~shift(k);
end

candidate = emptyCandidate();
candidate = appendPoint(candidate,x(1),y(1),theta(1),gear(1),0,true, ...
    1,source_s(1),'path_endpoint');
i = 1; run_id = 0;
while i<n
    edge = i+1;
    if moving_edge(edge)
        last_edge = edge;
        while last_edge<n && moving_edge(last_edge+1) && ~source_anchor(last_edge)
            last_edge = last_edge+1;
        end
        run_id = run_id+1;
        source_indices = i:last_edge;
        run = resampleRun(source_indices,x,y,theta,gear,source_s,cfg.resample_spacing_m,run_id);
        for j = 1:numel(run.x)
            candidate = appendPoint(candidate,run.x(j),run.y(j),run.theta(j), ...
                run.gear(j),run.run_id(j),run.anchor_mask(j),run.source_index(j), ...
                run.source_s_m(j),run.anchor_reason{j});
        end
        i = last_edge;
    else
        i = i+1;
        reason = 'stationary_or_gear_switch_anchor';
        if i==n, reason = 'path_endpoint'; end
        candidate = appendPoint(candidate,x(i),y(i),theta(i),gear(i),0,true, ...
            i,source_s(i),reason);
    end
end
report = struct('valid',numel(candidate.x)>=2,'reason','passed', ...
    'source_anchor_count',nnz(source_anchor),'moving_run_count',run_id);
if ~report.valid, report.reason = 'resampling_produced_insufficient_points'; end
end

function run = resampleRun(indices,x,y,theta,gear,source_s,spacing,run_id)
%RESAMPLERUN Interpolate XY and unwrapped course heading by source arc length.
idx = indices(:);
local_s = source_s(idx)-source_s(idx(1));
length_m = local_s(end);
count = max(2,ceil(length_m/spacing)+1);
sample_s = linspace(0,length_m,count).';
g = gear(idx(1));
course = unwrap(theta(idx)+(g<0)*pi);
run = struct();
run.x = interp1(local_s,x(idx),sample_s,'linear');
run.y = interp1(local_s,y(idx),sample_s,'linear');
course_sample = interp1(local_s,course,sample_s,'linear');
run.theta = wrapAngle(course_sample-(g<0)*pi);
run.gear = repmat(g,count,1);
run.run_id = repmat(run_id,count,1);
run.anchor_mask = false(count,1);
run.anchor_mask([1,end]) = true;
run.source_index = interp1(local_s,double(idx),sample_s,'linear');
run.source_s_m = source_s(idx(1))+sample_s;
run.anchor_reason = repmat({'resampled_moving_path'},count,1);
run.anchor_reason{1} = 'moving_run_start_anchor';
run.anchor_reason{end} = 'moving_run_end_anchor';
end

function candidate = appendPoint(candidate,x,y,theta,gear,run_id,anchor,source_index,source_s,reason)
%APPENDPOINT Append or assign a shared source anchor to a moving run.
if ~isempty(candidate.x) && abs(candidate.source_index(end)-source_index)<1e-10 && ...
        candidate.gear(end)==gear && candidate.run_id(end)==0 && run_id>0
    candidate.run_id(end) = run_id;
    candidate.anchor_mask(end) = true;
    candidate.reference_xy(end,:) = [x,y];
    candidate.reference_theta(end) = theta;
    candidate.anchor_reason{end} = reason;
    return;
end
candidate.x(end+1,1) = x;
candidate.y(end+1,1) = y;
candidate.theta(end+1,1) = theta;
candidate.gear(end+1,1) = gear;
candidate.run_id(end+1,1) = run_id;
candidate.anchor_mask(end+1,1) = logical(anchor);
candidate.source_index(end+1,1) = source_index;
candidate.source_s_m(end+1,1) = source_s;
candidate.reference_xy(end+1,:) = [x,y];
candidate.reference_theta(end+1,1) = theta;
candidate.anchor_reason{end+1,1} = reason;
end

function [solution,result] = solveRun(candidate,indices,corridor,vehicle,planner,cfg)
%SOLVERUN Assemble normalized quadratic smoothness/reference/spacing terms.
n = numel(indices);
xref = double(candidate.x(indices));
yref = double(candidate.y(indices));
pref = [xref,yref];
identity = eye(n);
d1 = diff(identity,1,1);
d2 = diff(identity,2,1);
scale = cfg.resample_spacing_m^2;
ws = cfg.weights.smoothness/max(1,n-2)/scale;
wr = cfg.weights.reference/max(1,n)/scale;
wl = cfg.weights.spacing/max(1,n-1)/scale;
H1 = 2*(ws*(d2'*d2)+wr*identity+wl*(d1'*d1));
H = blkdiag(H1,H1);
spacing_reference = [d1*xref;d1*yref];
f = [2*wr*(-xref)-2*wl*(d1'*d1*xref); ...
    2*wr*(-yref)-2*wl*(d1'*d1*yref)];
% The expression above is the exact gradient of the two squared residuals.
f = [f(1:n);f(n+1:end)];
lower = [corridor.lower(indices,1);corridor.lower(indices,2)];
upper = [corridor.upper(indices,1);corridor.upper(indices,2)];
anchors = logical(candidate.anchor_mask(indices));
for k = find(anchors(:)).'
    lower(k) = xref(k); upper(k) = xref(k);
    lower(n+k) = yref(k); upper(n+k) = yref(k);
end

t0 = [cos(candidate.theta(indices(1))+(candidate.gear(indices(1))<0)*pi), ...
    sin(candidate.theta(indices(1))+(candidate.gear(indices(1))<0)*pi)];
t1 = [cos(candidate.theta(indices(end))+(candidate.gear(indices(end))<0)*pi), ...
    sin(candidate.theta(indices(end))+(candidate.gear(indices(end))<0)*pi)];
n0 = [-t0(2),t0(1)]; n1 = [-t1(2),t1(1)];
Aeq = zeros(2,2*n); beq = zeros(2,1);
Aeq(1,2) = n0(1); Aeq(1,n+2) = n0(2);
beq(1) = dot(n0,pref(1,:));
Aeq(2,n-1) = n1(1); Aeq(2,2*n-1) = n1(2);
beq(2) = dot(n1,pref(end,:));
minimum_edge = cfg.corridor.minimum_neighbor_projection_m;
Aedge = zeros(2,2*n); bedge = -minimum_edge*ones(2,1);
Aedge(1,1) = t0(1); Aedge(1,n+1) = t0(2);
Aedge(1,2) = -t0(1); Aedge(1,n+2) = -t0(2);
Aedge(2,n-1) = t1(1); Aedge(2,2*n-1) = t1(2);
Aedge(2,n) = -t1(1); Aedge(2,2*n) = -t1(2);

solution = [xref;yref];
result = struct('success',false,'reason','solver_not_run','exitflag',NaN,'output',struct());
if ~strcmp(cfg.solver.name,'quadprog')
    result.reason = 'unsupported_solver';
    return;
end
try
    options = optimoptions('quadprog','Algorithm',cfg.solver.algorithm, ...
        'Display',cfg.solver.display,'MaxIterations',cfg.solver.max_iterations, ...
        'ConstraintTolerance',cfg.solver.constraint_tolerance, ...
        'OptimalityTolerance',cfg.solver.optimality_tolerance, ...
        'StepTolerance',cfg.solver.step_tolerance);
    z = [xref;yref];
    exitflag = NaN;
    output = struct();
    delta_max = min(vehicle.steering.maximum_steer_rad, ...
        planner.dynamics.delta_max_rad);
    kappa_max = cfg.weights.curvature_safety_factor* ...
        tan(delta_max)/vehicle.dimensions_m.wheelbase;
    for iteration = 1:cfg.weights.curvature_linearization_iterations
        [Acurvature,bcurvature] = curvatureConstraints( ...
            z,n,candidate,indices,kappa_max);
        A = [Aedge;Acurvature];
        b = [bedge;bcurvature];
        [z,~,exitflag,output] = quadprog((H+H')/2,f,A,b,Aeq,beq, ...
            lower,upper,z,options);
        if isempty(z) || exitflag<=0 || any(~isfinite(z)), break; end
    end
catch exception
    result.reason = ['solver_error:' exception.identifier];
    result.output = struct('message',exception.message);
    return;
end
result.exitflag = exitflag;
result.output = output;
if isempty(z) || exitflag<=0 || any(~isfinite(z))
    result.reason = 'solver_failed_or_infeasible';
    return;
end
violation = max([0;A*z-b;abs(Aeq*z-beq);lower-z;z-upper]);
if violation>10*cfg.solver.constraint_tolerance
    result.reason = 'solver_constraint_residual_exceeded';
    result.output.maximum_constraint_violation = violation;
    return;
end
solution = z;
result.success = true;
result.reason = 'solved';
result.output.maximum_constraint_violation = violation;
result.output.spacing_reference = spacing_reference;
result.output.curvature_linearization_iterations = iteration;
result.output.curvature_constraint_count = size(Acurvature,1);
end

function [A,b] = curvatureConstraints(z,n,candidate,indices,kappa_max)
%CURVATURECONSTRAINTS Linearize signed discrete curvature in reference axes.
A = zeros(2*max(0,n-2),2*n);
b = zeros(size(A,1),1);
x = z(1:n); y = z(n+1:end);
for j = 2:n-1
    before = [x(j)-x(j-1),y(j)-y(j-1)];
    after = [x(j+1)-x(j),y(j+1)-y(j)];
    tangent = before+after;
    tangent_norm = norm(tangent);
    if tangent_norm<=eps
        theta = candidate.theta(indices(j));
        tangent = [cos(theta+(candidate.gear(indices(j))<0)*pi), ...
            sin(theta+(candidate.gear(indices(j))<0)*pi)];
    else
        tangent = tangent/tangent_norm;
    end
    normal = [-tangent(2),tangent(1)];
    local_spacing = 0.5*(norm(before)+norm(after));
    bound = kappa_max*local_spacing^2;
    row = 2*(j-2)+1;
    A(row,[j-1,j,j+1]) = normal(1)*[1,-2,1];
    A(row,n+[j-1,j,j+1]) = normal(2)*[1,-2,1];
    A(row+1,:) = -A(row,:);
    b(row:row+1) = bound;
end
end

function value = pathSmoothness(candidate)
%PATHSMOOTHNESS Sum squared second differences over moving runs.
value = 0;
for run_id = unique(candidate.run_id(candidate.run_id>0)).'
    idx = find(candidate.run_id==run_id);
    if numel(idx)>=3
        dx2 = diff(candidate.x(idx),2);
        dy2 = diff(candidate.y(idx),2);
        value = value+sum(dx2.^2+dy2.^2);
    end
end
end

function value = spacingError(candidate)
%SPACINGERROR Measure vector deviations from the equal-arc reference steps.
value = 0;
for run_id = unique(candidate.run_id(candidate.run_id>0)).'
    idx = find(candidate.run_id==run_id);
    if numel(idx)>=2
        current = [diff(candidate.x(idx)),diff(candidate.y(idx))];
        reference = diff(candidate.reference_xy(idx,:),1,1);
        value = value+sum(sum((current-reference).^2,2));
    end
end
end

function candidate = updateCandidateHeading(candidate)
%UPDATECANDIDATEHEADING Recompute geometric tangent without dynamic timing.
for run_id = unique(candidate.run_id(candidate.run_id>0)).'
    idx = find(candidate.run_id==run_id);
    if numel(idx)<2, continue; end
    edge = diff([candidate.x(idx),candidate.y(idx)],1,1);
    course = unwrap(atan2(edge(:,2),edge(:,1)));
    body_heading = course-(candidate.gear(idx(1))<0)*pi;
    node_heading = [body_heading(1); ...
        (body_heading(1:end-1)+body_heading(2:end))/2;body_heading(end)];
    candidate.theta(idx) = wrapAngle(node_heading);
    anchored = candidate.anchor_mask(idx);
    candidate.theta(idx(anchored)) = candidate.reference_theta(idx(anchored));
end
end

function angle = wrapAngle(angle)
%WRAPANGLE Normalize headings to [-pi,pi).
angle = mod(angle+pi,2*pi)-pi;
end

function value = conditional(flag,a,b)
%CONDITIONAL Select a status string.
if flag, value = a; else, value = b; end
end
