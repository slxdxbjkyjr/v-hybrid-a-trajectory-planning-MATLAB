function report = vhybrid_validate_lateral_acceleration(path, vehicle, planner)
%VHYBRID_VALIDATE_LATERAL_ACCELERATION 独立逐段验收横向加速度硬约束。
% 第 k 个转角/加速度用于 k-1 到 k；速度为非负大小，gear 可为 +/-1。
% 不使用路径缓存的曲率、横向加速度或动力学 valid_flag。恒转角、恒加速度
% 段分别由记录的端点速度和 v_start+a*dt 重算峰值，取两者较大值，避免
% 篡低记录末速掩盖超限。模型末速与记录末速的不一致仍由完整重放器验收。
% 峰值位置/时间使用相应记录端点；model_exit 明确表示模型预测的出口速度。
% 关闭约束时直接返回 disabled 报告，不改变旧轨迹验收行为。
cfg = validate_lateral_acceleration_config(planner.dynamics);
template = segmentRecord();
report = struct('enabled',cfg.enabled,'valid',true,'reason','constraint_disabled', ...
    'max_lateral_acceleration_mps2',cfg.max_lateral_acceleration_mps2, ...
    'lateral_acceleration_tolerance_mps2',cfg.lateral_acceleration_tolerance_mps2, ...
    'checked_segment_count',0,'violation_count',0,'invalid_segment_count',0, ...
    'max_peak_abs_ay_mps2',0,'segments',repmat(template,0,1), ...
    'violations',repmat(template,0,1),'failure_reasons',{{}});
if ~cfg.enabled, return; end
report.reason = 'passed';
required = {'x','y','theta','v','t','acceleration','steering_angle'};
if ~isstruct(path) || ~isscalar(path) || ~all(isfield(path,required))
    report = invalidPath(report,'missing_or_invalid_path_fields');
    return;
end
n = numel(path.x);
if n < 2
    report = invalidPath(report,'insufficient_trajectory_samples');
    return;
end
for f = 1:numel(required)
    value = path.(required{f});
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || numel(value) ~= n
        report = invalidPath(report,'invalid_trajectory_field_shape_or_type');
        return;
    end
    % 验证类型后再转double，防止混合整型字段拼接时发生数值截断。
    path.(required{f}) = double(value(:));
end
if ~isstruct(vehicle) || ~isscalar(vehicle) || ~isfield(vehicle,'dimensions_m') || ...
        ~isstruct(vehicle.dimensions_m) || ~isscalar(vehicle.dimensions_m) || ...
        ~isfield(vehicle.dimensions_m,'wheelbase') || ...
        ~finiteScalar(vehicle.dimensions_m.wheelbase) || vehicle.dimensions_m.wheelbase <= 0
    report = invalidPath(report,'invalid_wheelbase');
    return;
end
gears = ones(n,1);
if isfield(path,'gear')
    if ~isnumeric(path.gear) || ~isreal(path.gear) || ~isvector(path.gear) || ...
            numel(path.gear) ~= n || any(~ismember(path.gear(:),[-1,1]))
        report = invalidPath(report,'invalid_gear');
        return;
    end
    gears = path.gear(:);
end
wheelbase = double(vehicle.dimensions_m.wheelbase);
segments = repmat(template,n-1,1);
threshold = cfg.max_lateral_acceleration_mps2+cfg.lateral_acceleration_tolerance_mps2;
for k = 2:n
    item = template;
    item.segment_index = k-1;
    item.start_index = k-1;
    item.end_index = k;
    item.time_start_s = double(path.t(k-1));
    item.time_end_s = double(path.t(k));
    item.position_start_xy = double([path.x(k-1),path.y(k-1)]);
    item.position_end_xy = double([path.x(k),path.y(k)]);
    item.heading_start_rad = double(path.theta(k-1));
    item.heading_end_rad = double(path.theta(k));
    item.speed_start_mps = double(path.v(k-1));
    item.speed_end_mps = double(path.v(k));
    item.acceleration_mps2 = double(path.acceleration(k));
    item.steering_angle_rad = double(path.steering_angle(k));
    item.gear = double(gears(k));
    dt = item.time_end_s-item.time_start_s;
    values = [item.time_start_s,item.time_end_s,item.position_start_xy, ...
        item.position_end_xy,item.heading_start_rad,item.heading_end_rad, ...
        item.speed_start_mps,item.speed_end_mps, ...
        item.acceleration_mps2,item.steering_angle_rad];
    if any(~isfinite(values)) || dt <= 0
        item.reason = 'invalid_segment_data_or_duration';
        report.invalid_segment_count = report.invalid_segment_count+1;
        segments(k-1) = item;
        continue;
    end
    item.model_speed_end_mps = item.speed_start_mps+item.acceleration_mps2*dt;
    item.curvature_per_m = tan(item.steering_angle_rad)/wheelbase;
    abs_curvature = abs(item.curvature_per_m);
    item.entry_abs_ay_mps2 = item.speed_start_mps^2*abs_curvature;
    item.exit_abs_ay_mps2 = item.speed_end_mps^2*abs_curvature;
    item.model_exit_abs_ay_mps2 = item.model_speed_end_mps^2*abs_curvature;
    item.stored_peak_abs_ay_mps2 = max(abs([item.speed_start_mps,item.speed_end_mps]))^2*abs_curvature;
    item.model_peak_abs_ay_mps2 = max(abs([item.speed_start_mps,item.model_speed_end_mps]))^2*abs_curvature;
    [item.peak_abs_ay_mps2,endpoint] = max( ...
        [item.entry_abs_ay_mps2,item.exit_abs_ay_mps2,item.model_exit_abs_ay_mps2]);
    item.peak_endpoint = 'entry';
    item.peak_time_s = item.time_start_s;
    item.peak_position_xy = item.position_start_xy;
    item.peak_speed_mps = item.speed_start_mps;
    if endpoint > 1
        item.peak_time_s = item.time_end_s;
        item.peak_position_xy = item.position_end_xy;
        item.peak_endpoint = 'exit';
        item.peak_speed_mps = item.speed_end_mps;
        if endpoint == 3
            item.peak_endpoint = 'model_exit';
            item.peak_speed_mps = item.model_speed_end_mps;
        end
    end
    item.entry_violation = item.entry_abs_ay_mps2 > threshold;
    item.exit_violation = item.exit_abs_ay_mps2 > threshold;
    item.model_exit_violation = item.model_exit_abs_ay_mps2 > threshold;
    lateral_values = [item.model_speed_end_mps,item.curvature_per_m, ...
        item.entry_abs_ay_mps2,item.exit_abs_ay_mps2,item.model_exit_abs_ay_mps2];
    if any(~isfinite(lateral_values))
        item.reason = 'nonfinite_lateral_acceleration';
        report.invalid_segment_count = report.invalid_segment_count+1;
    else
        item.valid = ~item.entry_violation && ~item.exit_violation && ~item.model_exit_violation;
        item.reason = 'passed';
        if item.entry_violation
            item.reason = 'entry_lateral_acceleration_exceeded';
        elseif item.exit_violation
            item.reason = 'exit_lateral_acceleration_exceeded';
        elseif item.model_exit_violation
            item.reason = 'model_exit_lateral_acceleration_exceeded';
        end
        report.violation_count = report.violation_count+double(~item.valid);
        report.max_peak_abs_ay_mps2 = max(report.max_peak_abs_ay_mps2,item.peak_abs_ay_mps2);
    end
    segments(k-1) = item;
end
report.checked_segment_count = n-1;
report.segments = segments;
report.violations = segments(~[segments.valid]);
report.failure_reasons = unique({report.violations.reason},'stable');
report.valid = report.violation_count==0 && report.invalid_segment_count==0;
if ~report.valid
    report.reason = 'lateral_acceleration_or_segment_validation_failed';
end
end

function report = invalidPath(report,reason)
%INVALIDPATH 保留明确失败原因，不把未执行的验收判为通过。
report.valid = false;
report.reason = reason;
report.invalid_segment_count = 1;
report.failure_reasons = {reason};
end

function flag = finiteScalar(value)
%FINITESCALAR 物理参数必须为有限实数标量，不能隐式接受字符或复数。
flag = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function item = segmentRecord()
%SEGMENTRECORD 段编号为 1-based，保留原始路径样本下标。
item = struct('segment_index',0,'start_index',0,'end_index',0, ...
    'time_start_s',NaN,'time_end_s',NaN,'position_start_xy',[NaN,NaN], ...
    'position_end_xy',[NaN,NaN],'heading_start_rad',NaN,'heading_end_rad',NaN, ...
    'speed_start_mps',NaN,'speed_end_mps',NaN, ...
    'model_speed_end_mps',NaN,'acceleration_mps2',NaN,'steering_angle_rad',NaN, ...
    'gear',1,'curvature_per_m',NaN,'entry_abs_ay_mps2',NaN,'exit_abs_ay_mps2',NaN, ...
    'model_exit_abs_ay_mps2',NaN,'stored_peak_abs_ay_mps2',NaN, ...
    'model_peak_abs_ay_mps2',NaN,'peak_abs_ay_mps2',NaN,'peak_time_s',NaN, ...
    'peak_position_xy',[NaN,NaN],'peak_speed_mps',NaN,'peak_endpoint','', ...
    'entry_violation',false,'exit_violation',false,'model_exit_violation',false, ...
    'valid',false,'reason','not_checked');
end
