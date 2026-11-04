function [phases, feasible, diagnostic] = vhybrid_segmented_speed_phases( ...
    segment_lengths_m, segment_curvatures_per_m, v0, vf, cruise_speed, ...
    dynamics, max_lateral_acceleration_mps2, tolerance_mps2, stop_distance_m, wait_time_s)
%VHYBRID_SEGMENTED_SPEED_PHASES Build a curvature-limited arc-length profile.
% PHASES rows are [duration_s, acceleration_mps2, end_speed_mps]. Curvature
% is constant over each geometry segment. A forward acceleration envelope
% and a backward braking envelope enforce local limits before phases are
% integrated by the existing vehicle model.

if nargin < 9, stop_distance_m = []; end
if nargin < 10, wait_time_s = 0; end
phases = zeros(0,3);
feasible = false;
diagnostic = struct('segment_lengths_m',zeros(1,0), ...
    'segment_curvatures_per_m',zeros(1,0),'local_speed_limits_mps',zeros(1,0), ...
    'boundary_speeds_mps',zeros(1,0),'profile_piece_lengths_m',zeros(1,0), ...
    'profile_piece_curvatures_per_m',zeros(1,0), ...
    'profile_piece_speed_limits_mps',zeros(1,0), ...
    'profile_piece_peak_speeds_mps',zeros(1,0), ...
    'required_braking_distance_m',0, ...
    'available_braking_distance_m',0,'reason','invalid_profile_input', ...
    'feasible',false,'lateral_constraint_caused_infeasibility',false, ...
    'cruise_speed_mps',cruise_speed,'stop_distance_m',NaN, ...
    'wait_time_s',wait_time_s);

if ~numericVector(segment_lengths_m) || ~numericVector(segment_curvatures_per_m) || ...
        numel(segment_lengths_m) ~= numel(segment_curvatures_per_m) || ...
        isempty(segment_lengths_m) || any(segment_lengths_m < 0) || ...
        any(segment_curvatures_per_m < 0) || ~finiteScalar(v0) || ...
        ~finiteScalar(vf) || ~finiteScalar(cruise_speed) || ...
        ~finiteScalar(max_lateral_acceleration_mps2) || ...
        max_lateral_acceleration_mps2 <= 0 || ~finiteScalar(tolerance_mps2) || ...
        tolerance_mps2 < 0 || ~isstruct(dynamics) || ...
        ~all(isfield(dynamics,{'a_max_mps2','a_min_mps2','v_min_mps','v_max_mps'}))
    return;
end
lengths = double(segment_lengths_m(:).');
curvatures = double(segment_curvatures_per_m(:).');
v0 = double(v0); vf = double(vf); cruise_speed = double(cruise_speed);
total_distance = sum(lengths);
diagnostic.segment_lengths_m = lengths;
diagnostic.segment_curvatures_per_m = curvatures;
distance_tolerance = 1e-10*max(1,total_distance);
if total_distance <= distance_tolerance
    diagnostic.reason = 'insufficient_trajectory_distance';
    return;
end

ap = double(dynamics.a_max_mps2);
ab = -double(dynamics.a_min_mps2);
vmin = double(dynamics.v_min_mps);
vmax = double(dynamics.v_max_mps);
speed_tolerance = 64*eps(max(1,vmax));
if ~all(isfinite([ap,ab,vmin,vmax])) || ap <= 0 || ab <= 0 || ...
        vmin < 0 || vmax < vmin || min([v0,vf,cruise_speed]) < vmin-speed_tolerance || ...
        max([v0,vf,cruise_speed]) > vmax+speed_tolerance
    diagnostic.reason = 'invalid_speed_or_acceleration_limits';
    return;
end
if ~isempty(stop_distance_m) && (~finiteScalar(stop_distance_m) || ...
        stop_distance_m <= distance_tolerance || ...
        stop_distance_m >= total_distance-distance_tolerance || ...
        ~finiteScalar(wait_time_s) || wait_time_s < 0)
    diagnostic.reason = 'invalid_stop_location_or_wait';
    return;
end

limits = vmax*ones(size(curvatures));
curved = curvatures > 0;
limits(curved) = min(vmax,sqrt((max_lateral_acceleration_mps2+ ...
    tolerance_mps2)./curvatures(curved)));
diagnostic.local_speed_limits_mps = limits;
first_curve = find(curved,1,'first');
first_nonzero = find(lengths > distance_tolerance,1,'first');
if ~isempty(first_curve) && v0 > limits(first_curve)+speed_tolerance
    diagnostic.required_braking_distance_m = max(0,(v0^2-limits(first_curve)^2)/(2*ab));
    diagnostic.available_braking_distance_m = sum(lengths(1:first_curve-1));
    if first_curve == first_nonzero
        diagnostic.reason = 'entry_speed_exceeds_curve_limit';
        diagnostic.lateral_constraint_caused_infeasibility = true;
        return;
    end
end
last_curve = find(curved,1,'last');
last_nonzero = find(lengths > distance_tolerance,1,'last');
if ~isempty(last_curve) && last_curve == last_nonzero && ...
        vf > limits(last_curve)+speed_tolerance
    diagnostic.reason = 'terminal_speed_exceeds_curve_limit';
    diagnostic.lateral_constraint_caused_infeasibility = true;
    return;
end

% Split geometry at every curvature transition and optional C-profile stop.
positions = [0,cumsum(lengths)];
if ~isempty(stop_distance_m)
    positions(end+1) = double(stop_distance_m);
    diagnostic.stop_distance_m = double(stop_distance_m);
end
positions = sort(positions);
unique_positions = positions(1);
for k = 2:numel(positions)
    if positions(k)-unique_positions(end) > distance_tolerance
        unique_positions(end+1) = positions(k); %#ok<AGROW>
    elseif abs(positions(k)-total_distance) <= distance_tolerance
        unique_positions(end) = total_distance;
    end
end
positions = unique_positions;
piece_lengths = diff(positions);
piece_curvatures = zeros(size(piece_lengths));
segment_ends = cumsum(lengths);
for k = 1:numel(piece_lengths)
    midpoint = 0.5*(positions(k)+positions(k+1));
    source_segment = find(midpoint <= segment_ends+distance_tolerance,1,'first');
    piece_curvatures(k) = curvatures(source_segment);
end
piece_limits = vmax*ones(size(piece_curvatures));
piece_curved = piece_curvatures > 0;
piece_limits(piece_curved) = min(vmax,sqrt((max_lateral_acceleration_mps2+ ...
    tolerance_mps2)./piece_curvatures(piece_curved)));
diagnostic.profile_piece_lengths_m = piece_lengths;
diagnostic.profile_piece_curvatures_per_m = piece_curvatures;
diagnostic.profile_piece_speed_limits_mps = piece_limits;
diagnostic.profile_piece_peak_speeds_mps = NaN(size(piece_lengths));
if any(piece_limits < vmin-speed_tolerance)
    diagnostic.reason = 'local_speed_limit_below_minimum_speed';
    diagnostic.lateral_constraint_caused_infeasibility = true;
    return;
end
boundary_limits = [piece_limits(1),min(piece_limits(1:end-1),piece_limits(2:end)), ...
    piece_limits(end)];
boundary_speeds = boundary_limits;
fixed = false(size(boundary_speeds));
fixed_values = NaN(size(boundary_speeds));
fixed([1,end]) = true;
fixed_values(1) = v0;
fixed_values(end) = vf;
if ~isempty(stop_distance_m)
    [~,stop_index] = min(abs(positions-double(stop_distance_m)));
    if fixed(stop_index) && abs(fixed_values(stop_index)) > speed_tolerance
        diagnostic.reason = 'stop_conflicts_with_endpoint_speed';
        return;
    end
    fixed(stop_index) = true;
    fixed_values(stop_index) = 0;
end
for k = find(fixed)
    if fixed_values(k) < vmin-speed_tolerance || ...
            fixed_values(k) > boundary_limits(k)+speed_tolerance
        if k == 1 && piece_curvatures(1) > 0
            diagnostic.reason = 'entry_speed_exceeds_curve_limit';
            diagnostic.lateral_constraint_caused_infeasibility = true;
        elseif k == numel(boundary_speeds) && piece_curvatures(end) > 0
            diagnostic.reason = 'terminal_speed_exceeds_curve_limit';
            diagnostic.lateral_constraint_caused_infeasibility = true;
        else
            diagnostic.reason = 'endpoint_speed_exceeds_local_limit';
            diagnostic.lateral_constraint_caused_infeasibility = ...
                boundary_limits(k) < vmax-speed_tolerance;
        end
        return;
    end
    boundary_speeds(k) = fixed_values(k);
end

% Maximum forward reachable envelope, resetting only at explicitly fixed anchors.
for k = 2:numel(boundary_speeds)
    reachable = sqrt(max(0,boundary_speeds(k-1)^2+2*ap*piece_lengths(k-1)));
    if fixed(k)
        if boundary_speeds(k) > reachable+speed_tolerance
            diagnostic.reason = 'insufficient_acceleration_distance';
            diagnostic.required_braking_distance_m = max(0, ...
                (boundary_speeds(k)^2-boundary_speeds(k-1)^2)/(2*ap));
            diagnostic.available_braking_distance_m = piece_lengths(k-1);
            return;
        end
    else
        boundary_speeds(k) = min(boundary_speeds(k),reachable);
    end
end
% Backward braking envelope slows on approach to each lower local limit.
for k = numel(boundary_speeds)-1:-1:1
    reachable = sqrt(max(0,boundary_speeds(k+1)^2+2*ab*piece_lengths(k)));
    if fixed(k)
        if boundary_speeds(k) > reachable+speed_tolerance
            diagnostic.reason = 'insufficient_braking_distance';
            diagnostic.required_braking_distance_m = max(0, ...
                (boundary_speeds(k)^2-boundary_speeds(k+1)^2)/(2*ab));
            diagnostic.available_braking_distance_m = piece_lengths(k);
            if ~isempty(first_curve) && first_curve > 1
                diagnostic.required_braking_distance_m = max(0, ...
                    (v0^2-limits(first_curve)^2)/(2*ab));
                diagnostic.available_braking_distance_m = sum(lengths(1:first_curve-1));
                diagnostic.lateral_constraint_caused_infeasibility = ...
                    diagnostic.required_braking_distance_m > ...
                    diagnostic.available_braking_distance_m+distance_tolerance;
            end
            return;
        end
    else
        boundary_speeds(k) = min(boundary_speeds(k),reachable);
    end
end
diagnostic.boundary_speeds_mps = boundary_speeds;

for k = 1:numel(piece_lengths)
    if piece_lengths(k) <= distance_tolerance, continue; end
    profile_cap = min(piece_limits(k),max([cruise_speed, ...
        boundary_speeds(k),boundary_speeds(k+1)]));
    [piece_phases,piece_feasible,piece_reason,piece_peak] = makeIntervalPhases( ...
        piece_lengths(k),boundary_speeds(k),boundary_speeds(k+1), ...
        profile_cap,ap,ab,vmin,vmax,distance_tolerance);
    if ~piece_feasible
        diagnostic.reason = piece_reason;
        diagnostic.lateral_constraint_caused_infeasibility = ...
            strcmp(piece_reason,'boundary_speed_exceeds_segment_limit') && ...
            piece_curvatures(k) > 0;
        return;
    end
    diagnostic.profile_piece_peak_speeds_mps(k) = piece_peak;
    phases = [phases;piece_phases]; %#ok<AGROW>
    if ~isempty(stop_distance_m) && abs(positions(k+1)-double(stop_distance_m)) <= distance_tolerance && ...
            wait_time_s > 0
        phases(end+1,:) = [double(wait_time_s),0,0]; %#ok<AGROW>
    end
end
if isempty(phases)
    diagnostic.reason = 'empty_speed_profile';
    return;
end
diagnostic.reason = 'passed';
diagnostic.feasible = true;
feasible = true;
end

function [phases, feasible, reason, peak] = makeIntervalPhases(distance,v0,vf,cap,ap,ab,vmin,vmax,tol)
%MAKEINTERVALPHASES Construct an exact accel/cruise/brake profile under a cap.
phases = zeros(0,3);
feasible = false;
reason = 'infeasible_segment_speed_profile';
peak = NaN;
if max(v0,vf) > cap+64*eps(max(1,vmax))
    reason = 'boundary_speed_exceeds_segment_limit';
    return;
end
if vf >= v0
    minimum_distance = (vf^2-v0^2)/(2*ap);
else
    minimum_distance = (v0^2-vf^2)/(2*ab);
end
if distance < minimum_distance-tol
    if v0 > vf
        reason = 'insufficient_braking_distance';
    else
        reason = 'insufficient_acceleration_distance';
    end
    return;
end
peak_squared = (2*ap*ab*distance+ab*v0^2+ap*vf^2)/(ap+ab);
peak = min(cap,sqrt(max(0,peak_squared)));
if peak < max(v0,vf)-64*eps(max(1,vmax))
    reason = 'boundary_speed_exceeds_reachable_envelope';
    return;
end
accel_distance = max(0,(peak^2-v0^2)/(2*ap));
brake_distance = max(0,(peak^2-vf^2)/(2*ab));
remaining = distance-accel_distance-brake_distance;
if remaining < -tol
    reason = 'infeasible_speed_envelope';
    return;
end
appendRamp(v0,peak,ap,ab);
if remaining > tol
    if peak <= max(vmin,1e-12)
        reason = 'zero_speed_cannot_cover_segment';
        phases = zeros(0,3);
        return;
    end
    phases(end+1,:) = [remaining/peak,0,peak]; %#ok<AGROW>
end
appendRamp(peak,vf,ap,ab);
if isempty(phases)
    if distance > tol
        reason = 'zero_speed_cannot_cover_segment';
        return;
    end
    feasible = true;
else
    actual_distance = 0;
    speed = v0;
    for k = 1:size(phases,1)
        dt = phases(k,1);
        next_speed = speed+phases(k,2)*dt;
        actual_distance = actual_distance+0.5*(speed+next_speed)*dt;
        speed = next_speed;
    end
    feasible = abs(actual_distance-distance) <= max(tol,1e-8*max(1,distance)) && ...
        abs(speed-vf) <= 1e-8*max(1,vmax);
    if ~feasible
        reason = 'profile_distance_or_terminal_speed_mismatch';
        phases = zeros(0,3);
    end
end
if feasible, reason = 'passed'; end

    function appendRamp(from_speed,to_speed,accel,brake)
        change = to_speed-from_speed;
        if abs(change) <= 1e-12, return; end
        if change > 0
            a = accel;
        else
            a = -brake;
        end
        phases(end+1,:) = [abs(change/a),a,to_speed]; %#ok<AGROW>
    end
end

function valid = numericVector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    all(isfinite(value(:)));
end

function valid = finiteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
