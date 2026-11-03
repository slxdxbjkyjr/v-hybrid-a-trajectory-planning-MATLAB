function cfg = validate_lateral_acceleration_config(dynamics)
%VALIDATE_LATERAL_ACCELERATION_CONFIG Validate the opt-in lateral hard limit.
% A missing enable field preserves Day 1-7 behavior. Enabling the constraint
% requires a finite positive limit. Tolerance is only for numeric roundoff;
% it cannot exceed 1e-6 m/s^2 and is never used to alter speed or steering.
if ~isstruct(dynamics) || ~isscalar(dynamics)
    error('LateralAcceleration:InvalidConfig','dynamics must be a scalar struct.');
end
cfg = struct('enabled',false,'max_lateral_acceleration_mps2',NaN, ...
    'lateral_acceleration_tolerance_mps2',1e-9);
field = 'lateral_acceleration_constraint_enabled';
if ~isfield(dynamics,field), return; end
enabled = dynamics.(field);
if ~islogical(enabled) || ~isscalar(enabled)
    error('LateralAcceleration:InvalidConfig', ...
        '%s must be a logical scalar.',field);
end
cfg.enabled = enabled;
if ~enabled, return; end
field = 'max_lateral_acceleration_mps2';
if ~isfield(dynamics,field) || ~positiveScalar(dynamics.(field))
    error('LateralAcceleration:InvalidConfig', ...
        'Enabled lateral constraint requires finite positive %s.',field);
end
cfg.max_lateral_acceleration_mps2 = double(dynamics.(field));
field = 'lateral_acceleration_tolerance_mps2';
if isfield(dynamics,field)
    tolerance = dynamics.(field);
    if ~isnumeric(tolerance) || ~isreal(tolerance) || ~isscalar(tolerance) || ...
            ~isfinite(tolerance) || tolerance < 0 || tolerance > 1e-6
        error('LateralAcceleration:InvalidConfig', ...
            '%s must be finite and in [0,1e-6] m/s^2.',field);
    end
    cfg.lateral_acceleration_tolerance_mps2 = double(tolerance);
end
end

function valid = positiveScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end
