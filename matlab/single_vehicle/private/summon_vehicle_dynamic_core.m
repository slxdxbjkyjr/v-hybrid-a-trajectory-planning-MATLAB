function [next_state, travelled_distance, curvature, valid_flag, lateral_detail] = ...
    summon_vehicle_dynamic_core(current_state,delta,acceleration,dt,wheelbase, ...
    v_max,v_min,a_min,a_max,delta_max,lateral_cfg)
%SUMMON_VEHICLE_DYNAMIC_CORE Shared average-speed bicycle integration.
% Caller supplies a lateral configuration already checked at the public entry.
next_state = double(current_state(:).');
travelled_distance = 0.0;
curvature = 0.0;
valid_flag = false;
lateral_detail = struct('enabled',lateral_cfg.enabled,'reason','not_evaluated');

if numel(current_state) ~= 5 || any(~isfinite(double(current_state)))
    return;
end
if ~all(isfinite([delta,acceleration,dt,wheelbase,v_max,v_min, ...
        a_min,a_max,delta_max]))
    return;
end
if dt <= 0 || wheelbase <= 0 || v_min > v_max || a_min > a_max || delta_max < 0
    return;
end
if acceleration < a_min || acceleration > a_max || abs(delta) > delta_max
    return;
end

x = double(current_state(1));
y = double(current_state(2));
theta = double(current_state(3));
v = double(current_state(4));
t = double(current_state(5));
v_next = v + acceleration * dt;
% Tolerate only floating-point residue at speed bounds, never a physical clip.
speed_roundoff = 64 * eps(max(1,max(abs([v_min,v_max]))));
if v_next < v_min-speed_roundoff || v_next > v_max+speed_roundoff
    return;
end
if abs(v_next-v_min) <= speed_roundoff, v_next = v_min; end
if abs(v_next-v_max) <= speed_roundoff, v_next = v_max; end

if lateral_cfg.enabled
    [lateral_valid,lateral_detail] = summon_lateral_acceleration_kernel( ...
        v,v_next,delta,wheelbase,lateral_cfg);
    if ~lateral_valid, return; end
end

travelled_distance = 0.5 * (v + v_next) * dt;
curvature = tan(delta) / wheelbase;
delta_theta = travelled_distance * curvature;
theta_next = wrapToPiLocal(theta + delta_theta);

if abs(curvature) < 1e-12
    x_next = x + travelled_distance * cos(theta);
    y_next = y + travelled_distance * sin(theta);
else
    radius = 1.0 / curvature;
    x_next = x + radius * (sin(theta_next) - sin(theta));
    y_next = y - radius * (cos(theta_next) - cos(theta));
end

next_state = [x_next,y_next,theta_next,v_next,t+dt];
valid_flag = true;
end

function angle = wrapToPiLocal(angle)
angle = mod(angle + pi,2*pi) - pi;
end
