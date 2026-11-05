function [valid, detail] = summon_lateral_acceleration(v_start,v_end,delta,wheelbase,dynamics)
%SUMMON_LATERAL_ACCELERATION Check an entire constant-steer/acceleration edge.
% Signed speeds and nonnegative speed magnitudes are both supported.
if nargin < 5, dynamics = struct(); end
lateral_cfg = validate_lateral_acceleration_config(dynamics);
[valid,detail] = summon_lateral_acceleration_kernel( ...
    v_start,v_end,delta,wheelbase,lateral_cfg);
end
