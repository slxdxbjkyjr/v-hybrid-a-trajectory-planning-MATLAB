function [phases, feasible] = vhybrid_speed_phases(distance, v0, vf, cruise_speed, dynamics)
%VHYBRID_SPEED_PHASES 为 Dubins 连接生成满足距离与加速度的速度剖面。
% phases 每行是 [持续时间, 加速度, 相位末速度]。
% cruise_speed 是巡航候选值，不是整段速度上限；父节点速度较高时仍可先减速。
% 距离不足以到达候选速度时，按运动学方程求实际峰值或谷值。

phases = zeros(0,3);
feasible = false;
values = [distance,v0,vf,cruise_speed];
if ~isnumeric(values) || ~isreal(values) || any(~isfinite(values)) || ...
        distance <= 1e-10 || v0 < 0 || vf < 0 || cruise_speed < 0 || ...
        ~isstruct(dynamics) || ~isfield(dynamics,'a_max_mps2') || ...
        ~isfield(dynamics,'a_min_mps2') || ~isfield(dynamics,'v_min_mps') || ...
        ~isfield(dynamics,'v_max_mps')
    return;
end
ap = dynamics.a_max_mps2;
ab = -dynamics.a_min_mps2;
vmin = dynamics.v_min_mps;
vmax = dynamics.v_max_mps;
if ~isfinite(ap) || ~isfinite(ab) || ap <= 0 || ab <= 0 || ...
        ~isfinite(vmin) || ~isfinite(vmax) || vmin < 0 || vmax < vmin || ...
        min(v0,vf) < vmin-1e-10 || max(v0,vf) > vmax+1e-10 || ...
        cruise_speed < vmin-1e-10 || cruise_speed > vmax+1e-10
    return;
end

% 即使采用最大可用制动/加速，端点速度之间也需要的最短距离。
if vf >= v0
    minimum_distance = (vf^2-v0^2)/(2*ap);
else
    minimum_distance = (v0^2-vf^2)/(2*ab);
end
distance_tolerance = 1e-10*max(1,distance);
if distance < minimum_distance-distance_tolerance
    return;
end

% 尝试从起始速度变化到候选巡航速度，再变化到终端速度。
ramp_distance = rampDistance(v0,cruise_speed,ap,ab) + ...
    rampDistance(cruise_speed,vf,ap,ab);
if distance >= ramp_distance-distance_tolerance
    remaining = max(0,distance-ramp_distance);
    appendRamp(v0,cruise_speed);
    if remaining > distance_tolerance
        if cruise_speed <= max(vmin,1e-12)
            return; % 静止速度不能承载剩余路程，交由其他巡航候选尝试。
        end
        phases(end+1,:) = [remaining/cruise_speed,0,cruise_speed]; %#ok<AGROW>
    end
    appendRamp(cruise_speed,vf);
    feasible = ~isempty(phases) && profileMatchesDistance(phases,v0,distance,distance_tolerance);
    if ~feasible, phases=zeros(0,3); end
    return;
end

% 距离不足以到达候选速度：改用同样端点速度和距离精确确定峰值/谷值。
low_endpoint = min(v0,vf);
high_endpoint = max(v0,vf);
if cruise_speed > high_endpoint+1e-12
    peak_squared = (2*ap*ab*distance + ab*v0^2 + ap*vf^2)/(ap+ab);
    peak = sqrt(max(0,peak_squared));
    if peak < high_endpoint-1e-9 || peak > cruise_speed+1e-9 || peak > vmax+1e-9
        return;
    end
    appendRamp(v0,peak);
    appendRamp(peak,vf);
elseif cruise_speed < low_endpoint-1e-12
    valley_squared = (ap*v0^2 + ab*vf^2 - 2*ap*ab*distance)/(ap+ab);
    if valley_squared < -1e-10, return; end
    valley = sqrt(max(0,valley_squared));
    if valley < cruise_speed-1e-9 || valley > low_endpoint+1e-9 || valley < vmin-1e-9
        return;
    end
    appendRamp(v0,valley);
    appendRamp(valley,vf);
else
    % 巡航速度处于端点速度区间时，两段最大斜率恰为端点最短距离。
    return;
end
feasible = ~isempty(phases) && profileMatchesDistance(phases,v0,distance,distance_tolerance);
if ~feasible, phases=zeros(0,3); end

    function appendRamp(from_speed,to_speed)
        % 按车辆允许的加速度符号生成一段恒加速度运动。
        delta = to_speed-from_speed;
        if abs(delta) <= 1e-12, return; end
        if delta > 0
            acceleration = ap;
        else
            acceleration = -ab;
        end
        duration = abs(delta/acceleration);
        phases(end+1,:) = [duration,acceleration,to_speed]; %#ok<AGROW>
    end
end

function distance = rampDistance(from_speed,to_speed,ap,ab)
%RAMPDISTANCE 以最大允许加速度完成单调速度变化所需的路程。
if to_speed >= from_speed
    distance = (to_speed^2-from_speed^2)/(2*ap);
else
    distance = (from_speed^2-to_speed^2)/(2*ab);
end
end

function matches = profileMatchesDistance(phases,initial_speed,expected,tolerance)
%PROFILEMATCHESDISTANCE 独立核验相位连续速度和总路程。
speed = initial_speed;
distance = 0;
matches = true;
for k = 1:size(phases,1)
    duration = phases(k,1); acceleration = phases(k,2); target = phases(k,3);
    next_speed = speed+acceleration*duration;
    segment_distance = speed*duration+0.5*acceleration*duration^2;
    if duration <= 0 || segment_distance < -tolerance || ...
            abs(next_speed-target)>1e-8*max(1,target)
        matches = false;
        return;
    end
    distance = distance+segment_distance;
    speed = next_speed;
end
matches = abs(distance-expected)<=max(tolerance,1e-8*max(1,expected));
end
