function report = diagnose_day7_lateral_constraints(source_root,output_directory)
%DIAGNOSE_DAY7_LATERAL_CONSTRAINTS Read-only diagnosis of saved Day 7 paths.
% This function loads the original MAT without searching or retiming. The
% report is a legacy diagnostic, never acceptance of a new constrained path.
% Only the explicitly separate output directory receives a JSON report.
if nargin < 1
    source_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
if nargin < 2
    output_directory = fullfile(source_root,'outputs','day7_lateral_constraint', ...
        'task1','baseline_diagnostic_matlab');
end
source_root = canonicalPath(source_root);
output_directory = canonicalPath(output_directory);
protected = {fullfile(source_root,'outputs','day7_four_vehicle'), ...
    fullfile(source_root,'outputs','day7_plan_c')};
for k = 1:numel(protected)
    if isWithin(output_directory,canonicalPath(protected{k}))
        error('LateralAcceleration:ProtectedBaseline', ...
            'Diagnostic output cannot be inside an original Day 7 output directory.');
    end
end
source_mat = fullfile(source_root,'outputs','day7_four_vehicle', ...
    'day7_four_vehicle_trajectories.mat');
config_dir = fullfile(source_root,'config');
vehicle_file = fullfile(config_dir,'vehicle_config_day7.json');
planner_file = fullfile(config_dir,'planner_config_day7.json');
scenario_file = fullfile(config_dir,'scenario_day7_four_vehicle.json');
sources = {source_mat,vehicle_file,planner_file,scenario_file};
before = cellfun(@sha256,sources,'UniformOutput',false);
saved = load(source_mat,'paths','result');
if ~isfield(saved,'paths') || ~iscell(saved.paths) || isempty(saved.paths) || ...
        ~isfield(saved,'result') || ~isfield(saved.result,'per_vehicle')
    error('LateralAcceleration:InvalidSavedResult','Saved paths/identity metadata are missing.');
end
vehicle = jsondecode(fileread(vehicle_file));
legacy_planner = jsondecode(fileread(planner_file));
scenario = jsondecode(fileread(scenario_file));
planner = legacy_planner;
planner.dynamics.lateral_acceleration_constraint_enabled = true;
planner.dynamics.max_lateral_acceleration_mps2 = 4.9;
planner.dynamics.lateral_acceleration_tolerance_mps2 = 1e-9;
validate_lateral_acceleration_config(planner.dynamics);
[~,order] = sort([scenario.vehicles.priority]);
vehicles = scenario.vehicles(order);
expected_ids = {'vehicle_001','vehicle_002','vehicle_004','vehicle_003'};
if ~isequal({vehicles.id},expected_ids) || numel(saved.paths) ~= numel(vehicles) || ...
        ~isequal({saved.result.per_vehicle.id},expected_ids)
    error('LateralAcceleration:BaselineScenarioMismatch','Saved/current Day 7 order does not match.');
end
template = struct('vehicle_id','','sample_count',0,'legacy_lateral_enabled',false, ...
    'lateral_diagnostic',struct());
records = repmat(template,numel(vehicles),1);
for k = 1:numel(vehicles)
    path = saved.paths{k};
    start = vehicles(k).start_pose;
    goal = vehicles(k).summon_goal;
    if numel(path.t)<2 || abs(path.t(1))>1e-9 || abs(path.v(1))>1e-9 || ...
            norm([path.x(1)-start.x_m,path.y(1)-start.y_m])>1e-6 || ...
            abs(wrapAngle(path.theta(1)-start.yaw_rad))>1e-6 || ...
            norm([path.x(end)-goal.x_m,path.y(end)-goal.y_m])>0.05+1e-8 || ...
            abs(wrapAngle(path.theta(end)-goal.yaw_rad))>0.017+1e-8
        error('LateralAcceleration:BaselineScenarioMismatch', ...
            'Saved trajectory for %s does not match current Day 7 poses.',vehicles(k).id);
    end
    records(k).vehicle_id = vehicles(k).id;
    records(k).sample_count = numel(path.t);
    legacy_cfg = validate_lateral_acceleration_config(legacy_planner.dynamics);
    records(k).legacy_lateral_enabled = legacy_cfg.enabled;
    records(k).lateral_diagnostic = vhybrid_validate_lateral_acceleration(path,vehicle,planner);
end
after = cellfun(@sha256,sources,'UniformOutput',false);
if ~isequal(before,after)
    error('LateralAcceleration:BaselineChanged','Baseline source hashes changed during diagnosis.');
end
report = struct('artifact_kind','legacy_saved_trajectory_diagnostic', ...
    'matlab_executed',true,'new_constrained_search_executed',false, ...
    'new_constrained_trajectory_accepted',false, ...
    'planning_order',{expected_ids},'source_files',{sources}, ...
    'source_sha256_before',{before},'source_sha256_after',{after}, ...
    'source_unchanged',true,'max_lateral_acceleration_mps2',4.9, ...
    'tolerance_mps2',1e-9,'per_vehicle',records);
if ~exist(output_directory,'dir'), mkdir(output_directory); end
report_file = fullfile(output_directory,'legacy_lateral_diagnostic.json');
fid = fopen(report_file,'w','n','UTF-8');
if fid < 0, error('LateralAcceleration:OutputOpenFailed','Cannot open %s.',report_file); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
end

function value = canonicalPath(value)
value = char(java.io.File(char(value)).getCanonicalPath());
end

function flag = isWithin(candidate,parent)
flag = strcmpi(candidate,parent) || startsWith(lower(candidate),lower([parent,filesep]));
end

function value = sha256(file)
fid = fopen(file,'rb');
if fid < 0, error('LateralAcceleration:SourceOpenFailed','Cannot read %s.',file); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid,Inf,'*uint8');
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(bytes);
value = lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end

function angle = wrapAngle(angle)
angle = mod(angle+pi,2*pi)-pi;
end
