function [next_state, travelled_distance, curvature, valid_flag, lateral_detail] = ...
    summon_vehicle_dynamic(current_state, delta, acceleration, dt, wheelbase, ...
    v_max, v_min, a_min, a_max, delta_max, dynamics)
%SUMMON_VEHICLE_DYNAMIC 带速度和时间的运动学自行车模型。
%
% dynamics 为可选集中动力学配置；省略或缺少启用字段时保持 Day 1-7 行为。
% 状态更新和平均速度积分由 private 核心实现，公开的旧接口保持不变。

if nargin < 11, dynamics = struct(); end
lateral_cfg = validate_lateral_acceleration_config(dynamics);
[next_state,travelled_distance,curvature,valid_flag,lateral_detail] = ...
    summon_vehicle_dynamic_core(current_state,delta,acceleration,dt,wheelbase, ...
    v_max,v_min,a_min,a_max,delta_max,lateral_cfg);
end
