function [valid, detail] = summon_lateral_acceleration_kernel( ...
    v_start,v_end,delta,wheelbase,cfg)
%SUMMON_LATERAL_ACCELERATION_KERNEL Evaluate an edge with validated settings.
detail = struct('enabled',cfg.enabled,'curvature_per_m',NaN, ...
    'entry_abs_ay_mps2',NaN,'exit_abs_ay_mps2',NaN,'peak_abs_ay_mps2',NaN, ...
    'max_lateral_acceleration_mps2',cfg.max_lateral_acceleration_mps2, ...
    'lateral_acceleration_tolerance_mps2',cfg.lateral_acceleration_tolerance_mps2, ...
    'reason','invalid_segment_input');
valid = false;
if ~finiteScalar(v_start) || ~finiteScalar(v_end) || ~finiteScalar(delta) || ...
        ~finiteScalar(wheelbase) || wheelbase <= 0
    return;
end
detail.curvature_per_m = tan(double(delta))/double(wheelbase);
detail.entry_abs_ay_mps2 = double(v_start)^2*abs(detail.curvature_per_m);
detail.exit_abs_ay_mps2 = double(v_end)^2*abs(detail.curvature_per_m);
detail.peak_abs_ay_mps2 = max(abs(double(v_start)),abs(double(v_end)))^2* ...
    abs(detail.curvature_per_m);
if ~all(isfinite([detail.curvature_per_m,detail.peak_abs_ay_mps2]))
    detail.reason = 'nonfinite_lateral_acceleration';
    return;
end
if ~cfg.enabled
    valid = true;
    detail.reason = 'constraint_disabled';
    return;
end
valid = detail.peak_abs_ay_mps2 <= cfg.max_lateral_acceleration_mps2 + ...
    cfg.lateral_acceleration_tolerance_mps2;
if valid
    detail.reason = 'passed';
elseif detail.entry_abs_ay_mps2 > cfg.max_lateral_acceleration_mps2 + ...
        cfg.lateral_acceleration_tolerance_mps2
    detail.reason = 'entry_lateral_acceleration_exceeded';
else
    detail.reason = 'segment_lateral_acceleration_exceeded';
end
end

function valid = finiteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
