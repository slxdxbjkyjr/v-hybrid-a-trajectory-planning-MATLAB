function result = run_day7_lateral_limit_task4(task_file)
%RUN_DAY7_LATERAL_LIMIT_TASK4 Re-search the frozen Day 7 scene in stages.
% Each stage starts with a fresh space-time map and searches its vehicles anew.

project_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
config_dir = fullfile(project_dir,'config');
if nargin < 1 || isempty(task_file)
    task_file = fullfile(config_dir,'day7_lateral_limit_task4.json');
else
    task_file = char(task_file);
    if ~isfile(task_file)
        task_file = fullfile(config_dir,task_file);
    end
end
task = jsondecode(fileread(task_file));
output_dir = fullfile(project_dir,strrep(task.output_directory,'/',filesep));
if ~exist(output_dir,'dir'), mkdir(output_dir); end
scenario = jsondecode(fileread(fullfile(config_dir,task.scenario_config)));
vehicle = jsondecode(fileread(fullfile(config_dir,task.vehicle_config)));
map_config = jsondecode(fileread(fullfile(config_dir,task.map_config)));
planner_base = jsondecode(fileread(fullfile(config_dir,task.planner_config)));
planner_base = applySearchOverrides(planner_base,task);
validateInputs(task,scenario,planner_base);

planner_base.vhybrid.heuristic_weight = planner_base.day6.search_heuristic_weight;
planner_base.vhybrid.heuristic_method = planner_base.day6.heuristic_method;
delta_max = min(vehicle.steering.maximum_steer_rad,planner_base.dynamics.delta_max_rad);
planner_base.vhybrid.minimum_turning_radius_m = ...
    vehicle.dimensions_m.wheelbase/tan(delta_max);
validate_st_occupancy_config(planner_base);
geometry = summon_map(map_config);
ordered_ids = task.planning_order(:).';
vehicles = selectVehicles(scenario.vehicles,ordered_ids);
[poses_valid,pose_report] = validateInitialPoses(vehicles,geometry,vehicle,planner_base);

snapshot = struct('task_config',task,'scenario',scenario,'planner_config',planner_base, ...
    'vehicle_config',vehicle,'map_config',map_config, ...
    'motion_model','existing average-speed integration; step definition unchanged');
writeJson(fullfile(output_dir,'effective_config_snapshot.json'),snapshot);
save(fullfile(output_dir,'effective_config_snapshot.mat'),'snapshot','-v7');
result = struct('valid',false,'reason','not_started','planning_order',{ordered_ids}, ...
    'pose_validation',pose_report,'stages',repmat(emptyStage(),numel(task.stages),1), ...
    'output_files',struct(),'effective_configuration',struct());

for stage_index = 1:numel(task.stages)
    stage_config = task.stages(stage_index);
    stage_vehicles = selectVehicles(vehicles,stage_config.vehicle_ids(:).');
    stage = emptyStage();
    stage.id = stage_config.id;
    stage.stage = stage_config.id;
    stage.vehicle_ids = stage_config.vehicle_ids(:).';
    stage.paths = cell(numel(stage_vehicles),1);
    stage.per_vehicle = repmat(emptyVehicleRecord(),numel(stage_vehicles),1);
    for k = 1:numel(stage_vehicles)
        stage.per_vehicle(k).id = stage_vehicles(k).id;
        stage.per_vehicle(k).priority = stage_vehicles(k).priority;
        stage.per_vehicle(k).reason = 'not_planned_after_earlier_failure';
    end
    planner = planner_base;
    planner.vhybrid.max_search_nodes = stage_config.max_search_nodes;
    planner.vhybrid.max_search_time_s = stage_config.max_search_time_s;
    stage.effective_search_budget = struct( ...
        'max_search_nodes_per_vehicle',planner.vhybrid.max_search_nodes, ...
        'max_search_time_s_per_vehicle',planner.vhybrid.max_search_time_s, ...
        'time_horizon_s',planner.st_occupancy.t_max, ...
        'speed_limit_mps',planner.dynamics.v_max_mps, ...
        'lateral_limit_mps2',planner.dynamics.max_lateral_acceleration_mps2);

    if ~poses_valid
        stage.reason = 'invalid_frozen_scene_initial_poses';
        stage.failure_diagnostics = pose_report;
    else
        % Rebuild all occupancy from this stage's new searches only.
        resource_planner = planner;
        resource_planner.st_occupancy.safety_distance_m = max( ...
            resource_planner.st_occupancy.safety_distance_m, ...
            planner.day6.minimum_safety_distance_m/2);
        resource_planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
        st_map = st_occupancy_map(resource_planner,geometry);
        [st_map,~] = mark_static_obstacles(st_map,geometry,vehicle);
        obstacles = emptyObstacles();
        for k = 1:numel(stage_vehicles)
            current_vehicle = stage_vehicles(k);
            [start_state,goal_state] = vehicleStates(current_vehicle);
            search_planner = plannerForVehicle(planner,current_vehicle.id);
            timer = tic;
            path = summon_vhybrid_astar(start_state,goal_state,geometry,vehicle, ...
                search_planner,st_map,obstacles);
            elapsed = toc(timer);
            validation = validatePath(path,start_state,goal_state,geometry,vehicle,search_planner);
            path.task4_validation = validation;
            % Search-node internals are not part of the deliverable trajectory.
            if isfield(path,'search_nodes'), path.search_nodes = path.search_nodes([]); end
            stage.paths{k} = path;
            stage.per_vehicle(k) = makeVehicleRecord(current_vehicle,path,validation,elapsed);
            stage.search_time_s = stage.search_time_s+elapsed;
            if ~path.valid || ~validation.valid
                stage.failure_vehicle_id = current_vehicle.id;
                stage.failure_diagnostics = path.failure_diagnostics;
                stage.reason = failureReason(path,validation);
                break;
            end
            % Immediately reserve the new trajectory and terminal parking hold.
            [st_map,~] = mark_trajectory_occupancy(st_map,path,vehicle,current_vehicle.id);
            obstacles(end+1,1) = dynamic_vehicle_obstacle( ...
                path,vehicle,current_vehicle.id,search_planner); %#ok<AGROW>
            st_map = prepare_dynamic_context(st_map,obstacles);
        end
    end
    [stage.pairwise_validation,stage.pairs_valid] = validatePairs( ...
        stage.paths,stage.vehicle_ids,geometry,vehicle,planner,resource_plannerFor(planner));
    stage.completed_searches = nnz(cellfun(@(p)isstruct(p) && isfield(p,'search_statistics'),stage.paths));
    all_paths = numel(stage.paths)==numel(stage.vehicle_ids) && ...
        all(cellfun(@(p)isstruct(p) && isfield(p,'valid') && p.valid,stage.paths));
    stage.valid = poses_valid && all_paths && stage.pairs_valid && all([stage.per_vehicle.valid]);
    if stage.valid, stage.reason = 'passed'; end
    stage.stage_elapsed_s = stage.search_time_s;
    stage_dir = fullfile(output_dir,stage.id);
    [stage.output_files,stage.metrics] = day7_task4_write_outputs( ...
        stage,stage.per_vehicle,geometry,vehicle,planner,stage_dir,stage.id);
    result.stages(stage_index) = stage;
end

final_stage = result.stages(end);
result.valid = final_stage.valid;
result.reason = final_stage.reason;
result.pairwise_validation = final_stage.pairwise_validation;
result.effective_configuration = struct('task_config_file',task_file, ...
    'lateral_acceleration_limit_mps2',planner_base.dynamics.max_lateral_acceleration_mps2, ...
    'immediate_start_required',planner_base.day6.require_immediate_start, ...
    'motion_model',snapshot.motion_model, ...
    'stage_budgets',{arrayfun(@(s)s.effective_search_budget,result.stages, ...
    'UniformOutput',false)});
result.output_files = struct('effective_config_json', ...
    fullfile(output_dir,'effective_config_snapshot.json'),'effective_config_mat', ...
    fullfile(output_dir,'effective_config_snapshot.mat'),'result_json', ...
    fullfile(output_dir,'day7_task4_result.json'),'result_mat', ...
    fullfile(output_dir,'day7_task4_result.mat'),'output_index', ...
    fullfile(output_dir,'output_index.json'),'stage_directories', ...
    {arrayfun(@(s)fullfile(output_dir,s.id),task.stages,'UniformOutput',false)});
[result.output_files.final_artifacts,result.final_metrics] = day7_task4_write_outputs( ...
    final_stage,final_stage.per_vehicle,geometry,vehicle, ...
    planner_base,output_dir,'day7_task4_final');
writeJson(result.output_files.result_json,compactResult(result));
save(result.output_files.result_mat,'result','snapshot','-v7.3');
index = struct('valid',result.valid,'reason',result.reason, ...
    'planning_order',{ordered_ids},'effective_configuration',result.effective_configuration, ...
    'stage_directories',{result.output_files.stage_directories}, ...
    'final_artifacts',result.output_files.final_artifacts, ...
    'result_json',result.output_files.result_json,'result_mat',result.output_files.result_mat, ...
    'effective_config_json',result.output_files.effective_config_json);
writeJson(result.output_files.output_index,index);
fprintf('Day7 task4: %s; output=%s\n',result.reason,output_dir);
end

function value = resource_plannerFor(planner)
%RESOURCE_PLANNERFOR Apply parking reservation semantics for independent checks.
value = planner;
value.st_occupancy.safety_distance_m = max(value.st_occupancy.safety_distance_m, ...
    planner.day6.minimum_safety_distance_m/2);
value.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
end

function validateInputs(task,scenario,planner)
%VALIDATEINPUTS Fail closed unless this is the requested constrained scene.
order = {'vehicle_001','vehicle_002','vehicle_004','vehicle_003'};
if ~isequal(task.planning_order(:).',order) || numel(scenario.vehicles)~=4 || ...
        ~all(ismember(order,{scenario.vehicles.id}))
    error('Day7Task4:ScenarioMismatch','任务四需要冻结四车场景和001→002→004→003顺序。');
end
v4 = scenario.vehicles(strcmp({scenario.vehicles.id},'vehicle_004'));
if any(abs([v4.summon_goal.x_m,v4.summon_goal.y_m,v4.summon_goal.yaw_rad]-[6,10,-3.14])>1e-12)
    error('Day7Task4:ScenarioMismatch','vehicle_004终点必须保持[6,10,-3.14]。');
end
constraint = validate_lateral_acceleration_config(planner.dynamics);
if ~constraint.enabled || abs(constraint.max_lateral_acceleration_mps2-4.9)>1e-12
    error('Day7Task4:LateralConstraintDisabled','任务四必须启用4.9 m/s^2横向限制。');
end
for k = 1:numel(task.stages)
    if isempty(task.stages(k).vehicle_ids) || task.stages(k).max_search_nodes<1 || ...
            task.stages(k).max_search_time_s<=0 || ~all(ismember(task.stages(k).vehicle_ids,order))
        error('Day7Task4:InvalidStage','任务四阶段配置或搜索预算无效。');
    end
end
end

function selected = selectVehicles(source,ids)
%SELECTVEHICLES Preserve the explicitly requested order.
selected = repmat(source(1),1,numel(ids));
for k = 1:numel(ids)
    idx = find(strcmp({source.id},ids{k}),1);
    if isempty(idx), error('Day7Task4:VehicleNotFound','缺少车辆%s。',ids{k}); end
    selected(k) = source(idx);
end
end

function [ok,report] = validateInitialPoses(vehicles,geometry,vehicle,planner)
%VALIDATEINITIALPOSES Check all configured poses and simultaneous starts.
ok = true;
checks = repmat(struct('vehicle_id','','kind','','valid',false,'detail',struct()), ...
    numel(vehicles)*2,1);
row = 0;
for k = 1:numel(vehicles)
    poses = {[vehicles(k).start_pose.x_m,vehicles(k).start_pose.y_m,vehicles(k).start_pose.yaw_rad], ...
        [vehicles(k).summon_goal.x_m,vehicles(k).summon_goal.y_m,vehicles(k).summon_goal.yaw_rad]};
    labels = {'start','goal'};
    for j = 1:2
        row = row+1;
        [hit,detail] = day6_static_collision_check(poses{j},geometry,vehicle);
        checks(row) = struct('vehicle_id',vehicles(k).id,'kind',labels{j}, ...
            'valid',~hit,'detail',detail);
        ok = ok && ~hit;
    end
end
pairs = repmat(struct('vehicle_a','','vehicle_b','','overlap',false,'minimum_distance_m',NaN), ...
    numel(vehicles)*(numel(vehicles)-1)/2,1);
row = 0;
for i = 1:numel(vehicles)-1
    for j = i+1:numel(vehicles)
        row = row+1;
        [hit,d] = check_vehicle_vehicle_collision(vehicleStates(vehicles(i)), ...
            vehicleStates(vehicles(j)),vehicle,vehicle,planner.day6);
        pairs(row) = struct('vehicle_a',vehicles(i).id,'vehicle_b',vehicles(j).id, ...
            'overlap',hit,'minimum_distance_m',d.minimum_distance_m);
        ok = ok && ~hit;
    end
end
report = struct('valid',ok,'pose_checks',checks,'initial_pairs',pairs);
end

function planner = plannerForVehicle(planner,id)
%PLANNERFORVEHICLE Apply existing per-vehicle search-order overrides.
if ~isfield(planner,'day7_vehicle_search_overrides') || ...
        ~isfield(planner.day7_vehicle_search_overrides,id), return; end
fields = fieldnames(planner.day7_vehicle_search_overrides.(id));
for k = 1:numel(fields)
    planner.vhybrid.(fields{k}) = planner.day7_vehicle_search_overrides.(id).(fields{k});
end
end

function planner = applySearchOverrides(planner,task)
%APPLYSEARCHOVERRIDES Apply isolated per-task search strategy overrides.
% Only search ordering/refinement fields are accepted; scene and safety
% constraints remain owned by the referenced planner and vehicle configs.
if ~isfield(task,'vehicle_search_overrides'), return; end
overrides = task.vehicle_search_overrides;
if ~isstruct(overrides) || ~isscalar(overrides)
    error('Day7Lateral:InvalidSearchOverrides', ...
        'vehicle_search_overrides 必须是按车辆ID组织的JSON对象。');
end
allowed = {'goal_connection_selection','solution_refinement_time_s', ...
    'heuristic_method','heuristic_weight', ...
    'goal_connection_duplicate_profile_pruning_enabled'};
vehicle_ids = fieldnames(overrides);
if ~isfield(planner,'day7_vehicle_search_overrides')
    planner.day7_vehicle_search_overrides = struct();
end
for k = 1:numel(vehicle_ids)
    id = vehicle_ids{k};
    fields = fieldnames(overrides.(id));
    for j = 1:numel(fields)
        name = fields{j};
        if ~any(strcmp(name,allowed))
            error('Day7Lateral:UnsupportedSearchOverride', ...
                '不允许通过任务覆盖字段 %s。',name);
        end
    end
    if ~isfield(planner.day7_vehicle_search_overrides,id)
        planner.day7_vehicle_search_overrides.(id) = struct();
    end
    patch = overrides.(id);
    for j = 1:numel(fields)
        name = fields{j};
        planner.day7_vehicle_search_overrides.(id).(name) = patch.(name);
    end
end
end

function [start,goal] = vehicleStates(vehicle)
%VEHICLESTATES Construct a new search start at t=0 and v=0.
start = [vehicle.start_pose.x_m,vehicle.start_pose.y_m,vehicle.start_pose.yaw_rad,0,0];
goal = [vehicle.summon_goal.x_m,vehicle.summon_goal.y_m,vehicle.summon_goal.yaw_rad,0,0];
end

function report = validatePath(path,start,goal,geometry,vehicle,planner)
%VALIDATEPATH Independently check replay, lateral and longitudinal limits.
report = struct('valid',false,'kinematic',struct(),'lateral',struct(), ...
    'longitudinal',struct(),'start_state',struct(),'terminal_state',struct(), ...
    'failure_reasons',{{}});
if ~path.valid || isempty(path.t)
    report.failure_reasons = {'search_did_not_return_valid_path'};
    return;
end
report.kinematic = vhybrid_validate_trajectory(path,geometry,vehicle,planner);
report.lateral = vhybrid_validate_lateral_acceleration(path,vehicle,planner);
acceleration = double(path.acceleration(:));
if numel(acceleration)>1, acceleration = acceleration(2:end); else, acceleration = 0; end
longitudinal_valid = all(isfinite(acceleration)) && ...
    all(acceleration>=planner.dynamics.a_min_mps2-1e-9) && ...
    all(acceleration<=planner.dynamics.a_max_mps2+1e-9);
report.longitudinal = struct('valid',longitudinal_valid, ...
    'minimum_acceleration_mps2',min(acceleration,[],'omitnan'), ...
    'maximum_acceleration_mps2',max(acceleration,[],'omitnan'), ...
    'configured_minimum_mps2',planner.dynamics.a_min_mps2, ...
    'configured_maximum_mps2',planner.dynamics.a_max_mps2);
tolerance = planner.vhybrid.kinematic_validation_tolerance;
start_valid = abs(path.t(1))<=tolerance && abs(path.v(1))<=tolerance && ...
    hypot(path.x(1)-start(1),path.y(1)-start(2))<=tolerance && ...
    abs(mod(path.theta(1)-start(3)+pi,2*pi)-pi)<=tolerance;
first_motion_time = NaN;
if numel(path.t)>1
    index = find(abs(path.v(2:end))>tolerance | hypot(diff(path.x),diff(path.y))>tolerance,1,'first');
    if ~isempty(index), first_motion_time = path.t(index+1); end
end
report.start_state = struct('valid',start_valid,'time_s',path.t(1),'speed_mps',path.v(1), ...
    'first_motion_time_s',first_motion_time,'immediate_start_required',planner.day6.require_immediate_start, ...
    'immediate_start',isfinite(first_motion_time) && first_motion_time<=tolerance);
terminal_valid = abs(path.v(end)-planner.vhybrid.terminal_speed_mps)<= ...
    planner.vhybrid.goal_speed_tolerance_mps && ...
    hypot(path.x(end)-goal(1),path.y(end)-goal(2))<=planner.vhybrid.goal_position_tolerance_m && ...
    abs(mod(path.theta(end)-goal(3)+pi,2*pi)-pi)<=planner.vhybrid.goal_heading_tolerance_rad;
report.terminal_state = struct('valid',terminal_valid,'time_s',path.t(end), ...
    'speed_mps',path.v(end),'goal_error_m',hypot(path.x(end)-goal(1),path.y(end)-goal(2)), ...
    'heading_error_rad',abs(mod(path.theta(end)-goal(3)+pi,2*pi)-pi));
checks = [report.kinematic.valid,report.lateral.valid,longitudinal_valid,start_valid,terminal_valid];
labels = {'kinematic_replay_failed','lateral_acceleration_exceeded', ...
    'longitudinal_acceleration_exceeded','invalid_initial_state','terminal_stop_or_goal_failed'};
report.failure_reasons = labels(~checks);
report.valid = all(checks);
end

function record = makeVehicleRecord(v,path,validation,elapsed)
%MAKEVEHICLERECORD Collect metrics from a searched path.
record = emptyVehicleRecord();
record.id = v.id;
record.priority = v.priority;
record.valid = path.valid && validation.valid;
record.planning_time_s = elapsed;
record.error_code = double(path.error_code);
record.search_statistics = path.search_statistics;
record.validation = validation;
record.failure_diagnostics = path.failure_diagnostics;
record.reason = 'passed';
if ~path.valid, record.reason = sprintf('search_error_%d',path.error_code); end
if ~validation.valid, record.reason = strjoin(validation.failure_reasons,','); end
if path.valid && ~isempty(path.t)
    if isfield(path,'travelled_distance'), record.path_length_m = sum(abs(path.travelled_distance));
    else, record.path_length_m = sum(hypot(diff(path.x),diff(path.y))); end
    record.arrival_time_s = path.t(end);
    record.maximum_speed_mps = max(abs(path.v));
    record.peak_lateral_acceleration_mps2 = validation.lateral.max_peak_abs_ay_mps2;
    record.longitudinal_acceleration_range_mps2 = ...
        [validation.longitudinal.minimum_acceleration_mps2, ...
        validation.longitudinal.maximum_acceleration_mps2];
end
end

function reason = failureReason(path,validation)
%FAILUREREASON Convert a failed search or validation into a stable summary.
% Keep validation evidence ahead of the planner error code so a path that was
% found but rejected during replay remains distinguishable from search failure.
if isstruct(validation) && isfield(validation,'failure_reasons') && ...
        ~isempty(validation.failure_reasons)
    reason = strjoin(validation.failure_reasons,',');
    return;
end
if isstruct(path) && isfield(path,'failure_diagnostics') && ...
        isstruct(path.failure_diagnostics) && ...
        isfield(path.failure_diagnostics,'reason') && ...
        ~isempty(path.failure_diagnostics.reason)
    reason = char(path.failure_diagnostics.reason);
    return;
end
if isstruct(path) && isfield(path,'error_code')
    reason = sprintf('search_error_%d',double(path.error_code));
else
    reason = 'search_failed';
end
end

function record = emptyVehicleRecord()
%EMPTYVEHICLERECORD Stable shape for planned and unplanned vehicle records.
record = struct('id','','priority',0,'valid',false,'reason','not_planned', ...
    'planning_time_s',0,'error_code',NaN,'path_length_m',NaN,'arrival_time_s',NaN, ...
    'maximum_speed_mps',NaN,'peak_lateral_acceleration_mps2',NaN, ...
    'longitudinal_acceleration_range_mps2',[NaN,NaN],'search_statistics',struct(), ...
    'validation',struct(),'failure_diagnostics',struct());
end

function [pairs,all_ok] = validatePairs(paths,ids,geometry,vehicle,planner,resource_planner)
%VALIDATEPAIRS Check swept rectangles and independent resource blocks to parking.
template = struct('vehicle_a','','vehicle_b','','collision',true,'conflict_reason','not_checked', ...
    'minimum_distance_m',NaN,'minimum_distance_lower_bound_m',NaN,'conflict_time_s',NaN, ...
    'conflict_position_xy',[NaN,NaN],'resource_overlap_blocks',NaN,'valid',false);
pairs = repmat(template,numel(ids)*(numel(ids)-1)/2,1);
row = 0; all_ok = true;
horizon = resource_planner.st_occupancy.t_max-max(1e-9,eps(resource_planner.st_occupancy.t_max));
for i = 1:numel(ids)-1
    for j = i+1:numel(ids)
        row = row+1; a = paths{i}; b = paths{j};
        if isempty(a) || isempty(b) || ~a.valid || ~b.valid
            pairs(row).vehicle_a = ids{i}; pairs(row).vehicle_b = ids{j};
            pairs(row).conflict_reason = 'path_missing_or_invalid';
            all_ok = false;
            continue;
        end
        states = [a.x(:),a.y(:),a.theta(:),a.v(:),a.t(:)];
        if states(end,5)<horizon
            last = states(end,:); last(4) = 0; last(5) = horizon;
            states(end+1,:) = last;
        end
        obstacle = dynamic_vehicle_obstacle(b,vehicle,ids{j},planner);
        [collision,detail] = check_trajectory_conflict(states,obstacle,vehicle,planner,[]);
        pair_map = st_occupancy_map(resource_planner,geometry);
        pair_map = mark_trajectory_occupancy(pair_map,a,vehicle,ids{i});
        [~,resource_stats] = mark_trajectory_occupancy(pair_map,b,vehicle,ids{j});
        pairs(row).vehicle_a = ids{i}; pairs(row).vehicle_b = ids{j};
        pairs(row).collision = collision; pairs(row).conflict_reason = detail.reason;
        pairs(row).minimum_distance_m = detail.minimum_distance_m;
        pairs(row).minimum_distance_lower_bound_m = detail.minimum_distance_lower_bound_m;
        pairs(row).conflict_time_s = detail.conflict_time_s;
        pairs(row).conflict_position_xy = detail.conflict_position_xy;
        pairs(row).resource_overlap_blocks = resource_stats.conflict_blocks;
        pairs(row).valid = ~collision && resource_stats.conflict_blocks==0;
        all_ok = all_ok && pairs(row).valid;
    end
end
end

function stage = emptyStage()
%EMPTYSTAGE Initialize consistent stage result fields.
stage = struct('id','','stage','','vehicle_ids',{{}},'valid',false,'reason','not_run', ...
    'failure_vehicle_id','','failure_diagnostics',struct(),'paths',{{}}, ...
    'per_vehicle',struct([]),'pairwise_validation',struct([]),'pairs_valid',false, ...
    'completed_searches',0,'search_time_s',0,'stage_elapsed_s',0, ...
    'effective_search_budget',struct(),'output_files',struct(),'metrics',struct());
end

function obstacles = emptyObstacles()
%EMPTYOBSTACLES Create an empty array compatible with dynamic_vehicle_obstacle.
obstacles = struct('id',{},'states',{},'vehicle_config',{},'footprint',{},'hold_end',{});
end

function output = compactResult(result)
%COMPACTRESULT Keep trajectories in MAT/CSV, not the machine-readable summary.
output = result;
for k = 1:numel(output.stages)
    output.stages(k).paths = cell(size(output.stages(k).paths));
    for j = 1:numel(output.stages(k).per_vehicle)
        record = output.stages(k).per_vehicle(j);
        if isfield(record,'search_statistics') && isfield(record.search_statistics,'conflict_examples')
            record.search_statistics.conflict_examples = {};
        end
        output.stages(k).per_vehicle(j) = record;
    end
end
end

function writeJson(file,value)
%WRITEJSON Write a UTF-8 JSON artifact.
fid = fopen(file,'w','n','UTF-8');
if fid<0, error('Day7Task4:OutputFailure','Cannot write %s.',file); end
cleanup = onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(value),'char');
end

