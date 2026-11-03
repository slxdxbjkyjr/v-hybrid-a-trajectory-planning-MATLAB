function [next_state, travelled_distance, curvature, valid_flag] = ...
    summon_vehicle_dynamic(current_state, delta, acceleration, dt, wheelbase, ...
    v_max, v_min, a_min, a_max, delta_max, dynamics)
%SUMMON_VEHICLE_DYNAMIC 带速度和时间的运动学自行车模型。
%
% 输入：
%   current_state - 当前运动状态 [x,y,theta,v,t]，单位 m、rad、m/s、s。
%   delta         - 前轮转角，单位 rad。
%   acceleration  - 纵向加速度 a，单位 m/s^2。
%   dt            - 时间步长，单位 s。
%   wheelbase     - 轴距 L，单位 m。
%   v_max/v_min   - 速度上限/下限，单位 m/s。
%   a_min/a_max   - 加速度下限/上限，单位 m/s^2。
%   delta_max     - 最大允许前轮转角绝对值，单位 rad。
%   dynamics      - 可选集中动力学配置；旧10参数接口不启用横向约束。
%
% 输出：
%   next_state        - 下一状态 [x,y,theta,v_next,t_next]。
%   travelled_distance- 本时间步行驶距离，单位 m。
%   curvature         - 曲率 tan(delta)/L，单位 1/m。
%   valid_flag        - 所有输入和运动约束均满足时为 true。
%
% 状态变量和算法逻辑：
%   该函数是原 APA VehicleDynamic.m 的独立扩展，不修改原函数。
%   先用 v_next=v+a*dt 更新速度，再用平均速度计算本步距离；
%   然后按曲率对车辆位姿积分，最后设置 t_next=t+dt。
%   任何约束违反均返回 valid_flag=false，并保持当前状态不变。

if nargin < 11, dynamics = struct(); end
% 配置错误不能被无效状态/动作的提前返回静默掩盖。
lateral_cfg = validate_lateral_acceleration_config(dynamics);
next_state = double(current_state(:).');
travelled_distance = 0.0;
curvature = 0.0;
valid_flag = false;

if numel(current_state) ~= 5 || any(~isfinite(double(current_state)))
    return;
end
if ~all(isfinite([delta, acceleration, dt, wheelbase, v_max, v_min, ...
        a_min, a_max, delta_max]))
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
% 精确制动末步可能出现机器精度级正/负残余，仅容忍浮点舍入；
% 明显超速/负速仍无效，不能用速度截断替代真实加减速。
speed_roundoff = 64 * eps(max(1, max(abs([v_min,v_max]))));
if v_next < v_min-speed_roundoff || v_next > v_max+speed_roundoff
    return;
end
if abs(v_next-v_min) <= speed_roundoff, v_next = v_min; end
if abs(v_next-v_max) <= speed_roundoff, v_next = v_max; end

if lateral_cfg.enabled
    lateral_valid = summon_lateral_acceleration(v,v_next,delta,wheelbase,dynamics);
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

next_state = [x_next, y_next, theta_next, v_next, t + dt];
valid_flag = true;
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将航向角归一化到 [-pi, pi)。
angle = mod(angle + pi, 2*pi) - pi;
end
