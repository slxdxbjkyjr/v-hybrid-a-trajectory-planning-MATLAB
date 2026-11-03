function report = vhybrid_validate_trajectory(path, map, vehicle, planner)
%VHYBRID_VALIDATE_TRAJECTORY 根据真实控制逐边重放，检查整条轨迹可执行性。
% 输入：带 x/y/theta/v/t、acceleration、steering_angle 的路径、地图、配置；
%       gear=+1/-1 是可选档位字段，v 是非负速度大小(m/s)，角度为 rad。
% 输出：valid、最大模型重放误差、最大航向变化、碰撞/约束失败数和换挡统计。
% 第 k 个控制量用于 k-1 到 k；角差按 [-pi,pi) 处理，避免 ±pi 伪跳变。
% 倒车边由独立档位适配器重放；换挡前后须停稳、车身不动且等待 >= 配置值。
% 连续静止边累计等待时间，兼容换挡等待段被输出采样细分的轨迹。
% 启用横向约束时独立计算每段入口/出口峰值；重放残差不混入横向拒绝。
report = struct('valid',false,'max_position_residual_m',0, ...
    'max_heading_residual_rad',0,'max_speed_residual_mps',0, ...
    'max_heading_step_rad',0,'collision_count',0,'constraint_failures',0, ...
    'gear_change_count',0,'gear_change_failures',0,'minimum_gear_change_wait_s',NaN);
report.failure_reasons = {};
report.lateral_acceleration = vhybrid_validate_lateral_acceleration(path,vehicle,planner);
if report.lateral_acceleration.enabled
    report.constraint_failures = report.lateral_acceleration.violation_count+ ...
        report.lateral_acceleration.invalid_segment_count;
    % 段输入本身非法时不能继续索引重放；配置错误已在独立验收器明确报错。
    report.failure_reasons = report.lateral_acceleration.failure_reasons;
    if report.lateral_acceleration.invalid_segment_count > 0, return; end
    % 横向验收已检查这些字段均为等长实数向量。仅新模式统一为double，
    % 避免整型时间/状态在重放算术中截断；旧模式保持原输入处理行为。
    fields = {'x','y','theta','v','t','acceleration','steering_angle'};
    for f = 1:numel(fields)
        value = path.(fields{f});
        path.(fields{f}) = double(value(:));
    end
    if isfield(path,'gear'), path.gear = double(path.gear(:)); end
end
if numel(path.x) < 2, return; end
dyn = planner.dynamics;
replay_planner = planner;
if report.lateral_acceleration.enabled
    replay_planner.dynamics.lateral_acceleration_constraint_enabled = false;
end
tol = planner.vhybrid.kinematic_validation_tolerance;
gears = ones(numel(path.x),1);
if isfield(path,'gear'), gears = path.gear(:); end
if numel(gears) ~= numel(path.x) || any(~ismember(gears,[-1,1]))
    report.constraint_failures = report.constraint_failures+1;
    report.gear_change_failures = 1;
    report.failure_reasons{end+1} = 'invalid_gear';
    return;
end
shift_flags = false(numel(path.x),1);
if report.lateral_acceleration.enabled && isfield(path,'is_gear_change')
    flags = path.is_gear_change;
    if ~(isnumeric(flags) || islogical(flags)) || ~isreal(flags) || ...
            ~isvector(flags) || numel(flags) ~= numel(path.x) || ...
            any(~ismember(flags(:),[0,1]))
        report.constraint_failures = report.constraint_failures+1;
        report.gear_change_failures = 1;
        report.failure_reasons{end+1} = 'invalid_gear_change_flags';
        return;
    end
end
if isfield(path,'is_gear_change'), shift_flags = logical(path.is_gear_change(:)); end
if numel(shift_flags) ~= numel(path.x)
    report.constraint_failures = report.constraint_failures+1;
    report.gear_change_failures = 1;
    report.failure_reasons{end+1} = 'invalid_gear_change_flag_shape';
    return;
end
stationary_wait_s = 0;
wait_limit = dyn.direction_change_time_s;
report.collision_count = double(summon_collision_check( ...
    [path.x(1),path.y(1),path.theta(1)],map,vehicle));
for k = 2:numel(path.x)
    start = [path.x(k-1),path.y(k-1),path.theta(k-1),path.v(k-1),path.t(k-1)];
    dt = path.t(k)-start(5);
    stationary = start(4) <= tol && path.v(k) <= tol && ...
        abs(path.acceleration(k)) <= tol && dt > 0 && ...
        hypot(path.x(k)-start(1),path.y(k)-start(2)) <= tol && ...
        abs(mod(path.theta(k)-start(3)+pi,2*pi)-pi) <= tol;
    if stationary
        stationary_wait_s = stationary_wait_s+dt;
    else
        stationary_wait_s = 0;
    end
    changed = gears(k) ~= gears(k-1);
    if changed || shift_flags(k)
        % 禁止在运动中直接改变方向；0.5s 为停稳后的换挡等待时间。
        report.gear_change_count = report.gear_change_count+double(changed);
        invalid_shift = ~changed || ~stationary || ...
            stationary_wait_s+64*eps(max(1,wait_limit)) < wait_limit;
        report.gear_change_failures = report.gear_change_failures+double(invalid_shift);
        if isnan(report.minimum_gear_change_wait_s)
            report.minimum_gear_change_wait_s = stationary_wait_s;
        else
            report.minimum_gear_change_wait_s = min(report.minimum_gear_change_wait_s,stationary_wait_s);
        end
        stationary_wait_s = 0;
    end
    [expected,~,~,valid] = summon_vehicle_dynamic_gear( ...
        start,path.steering_angle(k),path.acceleration(k),dt,vehicle,replay_planner,gears(k));
    report.constraint_failures = report.constraint_failures+double(~valid);
    report.max_position_residual_m = max(report.max_position_residual_m, ...
        hypot(expected(1)-path.x(k),expected(2)-path.y(k)));
    report.max_heading_residual_rad = max(report.max_heading_residual_rad, ...
        abs(mod(expected(3)-path.theta(k)+pi,2*pi)-pi));
    report.max_speed_residual_mps = max(report.max_speed_residual_mps,abs(expected(4)-path.v(k)));
    report.max_heading_step_rad = max(report.max_heading_step_rad, ...
        abs(mod(path.theta(k)-start(3)+pi,2*pi)-pi));
    report.collision_count = report.collision_count+double(summon_collision_check( ...
        [path.x(k),path.y(k),path.theta(k)],map,vehicle));
end
report.constraint_failures = report.constraint_failures+report.gear_change_failures;
report.valid = report.constraint_failures==0 && report.collision_count==0 && ...
    report.max_position_residual_m <= tol && ...
    report.max_heading_residual_rad <= tol && report.max_speed_residual_mps <= tol;
end
