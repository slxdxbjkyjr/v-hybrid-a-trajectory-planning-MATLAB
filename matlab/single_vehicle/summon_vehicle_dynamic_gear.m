function [next_state, travelled_distance, curvature, valid_flag, lateral_detail] = ...
    summon_vehicle_dynamic_gear(current_state, delta, acceleration, dt, vehicle, planner, gear)
%SUMMON_VEHICLE_DYNAMIC_GEAR 用独立档位适配器复用 Day3 有符号自行车模型。
% 输入：current_state=[x,y,theta,v,t]，v 始终是非负速度大小(m/s)，
%       theta 是车身航向(rad)，位置为后轴中心(m)，时间为 s；
%       delta 为前轮转角(rad)，acceleration 为速度大小的变化率(m/s^2)，
%       dt 为积分时间(s)，vehicle/planner 提供车辆和动力学约束；
%       gear=+1 表示前进，gear=-1 表示倒车，省略时默认前进。
% 输出：next_state 仍使用非负速度大小；travelled_distance 是带方向的
%       弧长(m)，curvature=tan(delta)/L(1/m)，valid_flag 为约束有效标志；
%       可选第五输出 lateral_detail 提供横向约束计算与拒绝原因。
% 逻辑：只在适配器内部把 v 和 a 乘以 gear，倒车时同步反转速度/加速度
%       区间，随后调用原 Day3 模型。车身航向始终连续且满足
%       theta_dot=gear*v*tan(delta)/L，不把倒车伪装成车身翻转 pi。
%       本函数只积分固定档位，停稳后换挡和等待时间由搜索与验收接口检查。
if nargin < 7, gear = 1; end
% 始终先验证启用配置，再处理动作有效性；缺少开关保持旧行为。
lateral_cfg = validate_lateral_acceleration_config(planner.dynamics);
next_state = double(current_state(:).');
travelled_distance = 0;
curvature = 0;
valid_flag = false;
lateral_detail = struct('enabled',lateral_cfg.enabled,'reason','not_evaluated');
if ~isnumeric(gear) || ~isreal(gear) || ~isscalar(gear) || (gear ~= 1 && gear ~= -1)
    return;
end
if ~isnumeric(current_state) || ~isreal(current_state) || numel(current_state) ~= 5 || ...
        any(~isfinite(current_state(:))) || current_state(4) < 0
    return;
end
dyn = planner.dynamics;
% 可选倒车限速只约束倒车，不改变原 Day3 模型或前进速度上限。
if gear == -1 && isfield(planner.vhybrid,'max_reverse_speed_mps')
    reverse_limit = planner.vhybrid.max_reverse_speed_mps;
    if ~isnumeric(reverse_limit) || ~isreal(reverse_limit) || ~isscalar(reverse_limit) || ...
            ~isfinite(reverse_limit) || reverse_limit <= 0
        return;
    end
    dyn.v_max_mps = min(dyn.v_max_mps,reverse_limit);
end
if dyn.v_min_mps < 0 || dyn.v_min_mps > dyn.v_max_mps || dyn.a_min_mps2 > dyn.a_max_mps2
    return;
end
roundoff = 64*eps(max(1,dyn.v_max_mps));
if current_state(4) < dyn.v_min_mps-roundoff || current_state(4) > dyn.v_max_mps+roundoff
    return;
end
signed_state = next_state;
signed_state(4) = gear*signed_state(4);
% 档位只有两个值，直接选有符号约束区间，避免每次积分调用通用 sort。
if gear == 1
    v_min = dyn.v_min_mps; v_max = dyn.v_max_mps;
    a_min = dyn.a_min_mps2; a_max = dyn.a_max_mps2;
else
    v_min = -dyn.v_max_mps; v_max = -dyn.v_min_mps;
    a_min = -dyn.a_max_mps2; a_max = -dyn.a_min_mps2;
end
[next_state,travelled_distance,curvature,valid_flag,lateral_detail] = ...
    summon_vehicle_dynamic_core( ...
    signed_state,delta,gear*acceleration,dt,vehicle.dimensions_m.wheelbase, ...
    v_max,v_min,a_min,a_max, ...
    min(vehicle.steering.maximum_steer_rad,dyn.delta_max_rad),lateral_cfg);
if valid_flag
    next_state(4) = abs(next_state(4));
else
    % 无效动作返回原公共状态，不暴露适配器内部的负速度表示。
    next_state = double(current_state(:).');
end
end
