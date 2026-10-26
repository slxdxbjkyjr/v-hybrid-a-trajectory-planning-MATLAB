function result = run_day7_sequential_planning_demo(save_outputs, options)
%RUN_DAY7_SEQUENTIAL_PLANNING_DEMO 复用Day6搜索器执行Day7四车顺序规划。
% 输入：save_outputs为是否写Day7独立结果；options.fault_event可选故障停车事件。
% 故障事件需要cause、failed_vehicle_id和affected_vehicles；受影响车记录
% 需要id、prefix_path、stopped_state=[x,y,theta,0,t]，前缀必须由有效控制生成。
% 输出：result包含四车路径/搜索统计、全车对连续碰撞与独立资源块检查、计时。
% 故障重规划按Day7排序：未受影响车辆先规划，受影响车辆使用已验证安全停车
% 前缀，再依故障后优先级从停车点规划剩余路径。所有资源均在Day7副本中处理。

if nargin < 1, save_outputs = true; end
if nargin < 2, options = struct(); end
if ~islogical(save_outputs) || ~isscalar(save_outputs)
    error('Day7:InvalidArgument','save_outputs必须是逻辑标量。');
end
if ~isstruct(options) || ~isscalar(options)
    error('Day7:InvalidArgument','options必须是标量结构体。');
end

total_timer = tic;
this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(this_dir));
config_dir = fullfile(project_dir,'config');
output_dir = fullfile(project_dir,'outputs','day7_four_vehicle');
result = struct('valid',false,'stage','day7_four_vehicle_sequential_planning', ...
    'reason','not_started','planning_order',{{}},'fault_event_summary',struct(), ...
    'pose_validation',struct(),'per_vehicle',struct([]),'paths',{{}}, ...
    'pairwise_validation',struct([]),'timings',struct(),'output_files',struct());
geometry=geometryOrEmpty(); vehicle=vehicleOrEmpty();

try
    vehicle = jsondecode(fileread(fullfile(config_dir,'vehicle_config_day7.json')));
    map_config = jsondecode(fileread(fullfile(config_dir,'map_config_day7.json')));
    planner = jsondecode(fileread(fullfile(config_dir,'planner_config_day7.json')));
    scenario = jsondecode(fileread(fullfile(config_dir,'scenario_day7_four_vehicle.json')));
    validateScenario(scenario);

    % 保持Day6代价与碰撞判据，由Day7隔离配置提供参数。
    planner.vhybrid.heuristic_weight = planner.day6.search_heuristic_weight;
    planner.vhybrid.heuristic_method = planner.day6.heuristic_method;
    delta_limit=vehicle.steering.maximum_steer_rad;
    if isfield(planner.dynamics,'delta_max_rad')
        delta_limit=min(delta_limit,planner.dynamics.delta_max_rad);
    end
    planner.vhybrid.minimum_turning_radius_m = vehicle.dimensions_m.wheelbase/tan(delta_limit);
    validate_st_occupancy_config(planner);
    geometry = summon_map(map_config);
    [ordered_vehicles,original_order] = sortVehicles(scenario.vehicles);
    n = numel(ordered_vehicles);
    result.planning_order = {ordered_vehicles.id};
    result.timings = struct('configuration_s',toc(total_timer), ...
        'per_vehicle_s',zeros(n,1),'planning_s',0,'pairwise_validation_s',0, ...
        'plot_s',0,'animation_s',0,'table_output_s',0, ...
        'file_output_s',0,'total_runtime_s',0);
    result.per_vehicle = repmat(vehicleRecord(),n,1);
    result.paths = cell(n,1);
    % 即使位姿或后续配置检查失败，状态表仍能标识全部车辆及其起终点。
    for k = 1:n
        result.per_vehicle(k) = vehicleRecord(ordered_vehicles(k));
        [start_state,goal_state] = vehicleStates(ordered_vehicles(k));
        result.per_vehicle(k).start_state = start_state;
        result.per_vehicle(k).goal_state = goal_state;
    end

    [pose_ok,pose_report] = validatePoses(ordered_vehicles,geometry,vehicle,planner);
    result.pose_validation = pose_report;
    if ~pose_ok
        result.reason = 'invalid_pose_or_initial_overlap';
        result = finishDay7(result,save_outputs,output_dir,total_timer,geometry,vehicle);
        return;
    end

    % 地图内静态配置和停车驻留规则均使用Day7副本。
    resource_planner = planner;
    resource_planner.st_occupancy.safety_distance_m = max( ...
        resource_planner.st_occupancy.safety_distance_m, ...
        planner.day6.minimum_safety_distance_m/2);
    resource_planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
    static_map = st_occupancy_map(resource_planner,geometry);
    [static_map,~] = mark_static_obstacles(static_map,geometry,vehicle);

    if isfield(options,'fault_event') && ~isempty(options.fault_event)
        [fault_ok,fault_data,fault_message] = validateFaultEvent(options.fault_event, ...
            ordered_vehicles,geometry,vehicle,planner);
        if ~fault_ok
            result.reason = 'invalid_fault_event';
            result.fault_event_summary = struct('valid',false,'message',fault_message);
            result = finishDay7(result,save_outputs,output_dir,total_timer,geometry,vehicle);
            return;
        end
        result.fault_event_summary = struct('valid',true,'cause',fault_data.cause, ...
            'failed_vehicle_id',fault_data.failed_vehicle_id, ...
            'affected_vehicle_ids',{fault_data.affected_ids}, ...
            'safe_stop_times',fault_data.stop_times);
        [fault_order,assigned_priority] = day7_fault_replan_order( ...
            ordered_vehicles,fault_data.affected_ids);
        fault_data.replan_ids=fault_order(ismember(fault_order,fault_data.affected_ids));
        result.planning_order = fault_order;
        for k = 1:n
            result.per_vehicle(k).effective_priority = assigned_priority(k);
        end
        [plan_ok,result] = planFaultScenario(result,ordered_vehicles,original_order, ...
            fault_data,static_map,geometry,vehicle,planner);
    else
        [plan_ok,result] = planNominalScenario(result,ordered_vehicles,static_map, ...
            geometry,vehicle,planner);
    end

    if ~plan_ok
        pair_timer = tic;
        [pairs,~] = validateAllPairs(result.paths,{ordered_vehicles.id}, ...
            geometry,vehicle,planner,resource_planner);
        result.pairwise_validation = pairs;
        result.timings.pairwise_validation_s = toc(pair_timer);
        result.reason = 'vehicle_search_failed';
        result = finishDay7(result,save_outputs,output_dir,total_timer,geometry,vehicle);
        return;
    end
    pair_timer = tic;
    [pairs,pairs_ok] = validateAllPairs(result.paths,{ordered_vehicles.id}, ...
        geometry,vehicle,planner,resource_planner);
    result.pairwise_validation = pairs;
    result.timings.pairwise_validation_s = toc(pair_timer);
    result.valid = pairs_ok && all([result.per_vehicle.valid]);
    if result.valid, result.reason = 'passed';
    else, result.reason = 'pairwise_collision_or_resource_overlap'; end
catch exception
    % 配置、接口或故障前缀错误均写成可定位的Day7失败结果。
    result.valid = false;
    result.reason = 'day7_runner_error';
    result.failure = struct('identifier',exception.identifier,'message',exception.message, ...
        'stack',exception.stack);
end
result = finishDay7(result,save_outputs,output_dir,total_timer,geometry,vehicle);
end

function [ok,result] = planNominalScenario(result,vehicles,static_map,geometry,vehicle,planner)
%PLANNOMINALSCENARIO 按初始priority逐辆规划并立刻预留真实轨迹。
ok = false; st_map = static_map; obstacles = emptyObstacles();
for k = 1:numel(vehicles)
    [start_state,goal_state] = vehicleStates(vehicles(k));
    timer = tic;
    search_planner = plannerForVehicle(planner,vehicles(k).id);
    path = summon_vhybrid_astar(start_state,goal_state,geometry,vehicle,search_planner,st_map,obstacles);
    elapsed = toc(timer);
    result.timings.per_vehicle_s(k) = elapsed;
    result.timings.planning_s = result.timings.planning_s+elapsed;
    result = putPath(result,k,path,start_state,goal_state,elapsed,vehicles(k).priority);
    if ~path.valid
        result.per_vehicle(k).failure_diagnostics = path.failure_diagnostics;
        result.per_vehicle(k).reason = sprintf('search_error_%d',path.error_code);
        result.failure = summarizeSearchFailure(vehicles(k).id,path);
        return;
    end
    [st_map,~] = mark_trajectory_occupancy(st_map,path,vehicle,vehicles(k).id);
    obstacle = dynamic_vehicle_obstacle(path,vehicle,vehicles(k).id,planner);
    obstacles(end+1,1) = obstacle; %#ok<AGROW>
    st_map = prepare_dynamic_context(st_map,obstacles);
end

ok = true;
end

function planner = plannerForVehicle(planner,vehicle_id)
%PLANNERFORVEHICLE 应用 Day7 显式声明的逐车启发函数覆盖。
% 覆盖只改变搜索节点排序，不改变运动学、代价权重或碰撞判据。
if ~isfield(planner,'day7_vehicle_search_overrides') || ...
        ~isstruct(planner.day7_vehicle_search_overrides) || ...
        ~isfield(planner.day7_vehicle_search_overrides,vehicle_id)
    return;
end
override = planner.day7_vehicle_search_overrides.(vehicle_id);
if ~isstruct(override) || ~isscalar(override) || isempty(fieldnames(override))
    error('Day7:InvalidVehicleSearchOverride', ...
        '逐车覆盖必须为非空标量结构体。');
end
allowed = {'heuristic_method','goal_connection_selection', ...
    'solution_refinement_time_s','goal_connection_cost_pruning_enabled', ...
    'heuristic_weight','reverse_penalty'};
unknown = setdiff(fieldnames(override),allowed);
if ~isempty(unknown)
    error('Day7:InvalidVehicleSearchOverride', ...
        '逐车搜索覆盖包含未知字段：%s。',strjoin(unknown,', '));
end
if isfield(override,'heuristic_method')
    method = override.heuristic_method;
    if ~((ischar(method) && isrow(method)) || (isstring(method) && isscalar(method))) || ...
            ~any(strcmp(char(method),{'dubins','euclidean','kinematic_lower_bound'}))
        error('Day7:InvalidVehicleSearchOverride', ...
            '逐车 heuristic_method 必须为 dubins、euclidean 或 kinematic_lower_bound。');
    end
    planner.vhybrid.heuristic_method = char(method);
end
if isfield(override,'goal_connection_selection')
    selection = override.goal_connection_selection;
    if ~((ischar(selection) && isrow(selection)) || ...
            (isstring(selection) && isscalar(selection))) || ...
            ~any(strcmp(char(selection),{'first_feasible','lowest_cost'}))
        error('Day7:InvalidVehicleSearchOverride', ...
            'goal_connection_selection 必须为 first_feasible 或 lowest_cost。');
    end
    planner.vhybrid.goal_connection_selection = char(selection);
end
if isfield(override,'solution_refinement_time_s')
    budget = override.solution_refinement_time_s;
    if ~isnumeric(budget) || ~isreal(budget) || ~isscalar(budget) || ...
            ~isfinite(budget) || budget < 0
        error('Day7:InvalidVehicleSearchOverride', ...
            'solution_refinement_time_s 必须为有限且非负的实数标量。');
    end
    planner.vhybrid.solution_refinement_time_s = double(budget);
end
if isfield(override,'goal_connection_cost_pruning_enabled')
    pruning = override.goal_connection_cost_pruning_enabled;
    if ~islogical(pruning) || ~isscalar(pruning)
        error('Day7:InvalidVehicleSearchOverride', ...
            'goal_connection_cost_pruning_enabled 必须为逻辑标量。');
    end
    planner.vhybrid.goal_connection_cost_pruning_enabled = pruning;
end
if isfield(override,'heuristic_weight')
    weight = override.heuristic_weight;
    if ~isnumeric(weight) || ~isreal(weight) || ~isscalar(weight) || ...
            ~isfinite(weight) || weight <= 0
        error('Day7:InvalidVehicleSearchOverride', ...
            'heuristic_weight 必须为正有限实数标量。');
    end
    planner.vhybrid.heuristic_weight = double(weight);
end
if isfield(override,'reverse_penalty')
    penalty = override.reverse_penalty;
    if ~isnumeric(penalty) || ~isreal(penalty) || ~isscalar(penalty) || ...
            ~isfinite(penalty) || penalty < 0
        error('Day7:InvalidVehicleSearchOverride', ...
            'reverse_penalty 必须为有限且非负实数标量。');
    end
    planner.vhybrid.reverse_penalty = double(penalty);
end
end

function failure = summarizeSearchFailure(vehicle_id,path)
%SUMMARIZESEARCHFAILURE 提取首个动态冲突，保留完整搜索统计供审计。
failure = struct('vehicle_id',vehicle_id,'error_code',double(path.error_code), ...
    'search_reason','','conflict_reason','none_recorded','conflict_vehicle_id','', ...
    'conflict_time_s',NaN,'conflict_position_xy',[NaN,NaN], ...
    'search_statistics',path.search_statistics,'failure_diagnostics',path.failure_diagnostics);
if isfield(path.failure_diagnostics,'reason')
    failure.search_reason = path.failure_diagnostics.reason;
end
examples = {};
if isfield(path.search_statistics,'conflict_examples')
    examples = path.search_statistics.conflict_examples;
end
for k = 1:numel(examples)
    example = examples{k};
    if ~isstruct(example) || ~isfield(example,'collision') || ~example.collision, continue; end
    failure.conflict_reason = example.reason;
    if isfield(example,'obstacle_id'), failure.conflict_vehicle_id = example.obstacle_id; end
    if isfield(example,'conflict_time_s'), failure.conflict_time_s = example.conflict_time_s; end
    if isfield(example,'conflict_position_xy'), failure.conflict_position_xy = example.conflict_position_xy; end
    break;
end
end

function [ok,result] = planFaultScenario(result,vehicles,original_order,fault, ...
    static_map,geometry,vehicle,planner)
%PLANFAULTSCENARIO 先规划未受影响车辆，再按降级优先级规划受影响车的后缀。
ok = false; affected = fault.affected_ids;
for k = 1:numel(vehicles)
    if ismember(vehicles(k).id,affected)
        result.paths{k} = fault.prefix_paths{k};
        result.per_vehicle(k).prefix_point_count = numel(fault.prefix_paths{k}.t);
    end
end

% 所有故障车前缀和安全停车驻留先作为约束加入，防止其他车辆穿过事故现场。
st_map = static_map; obstacles = emptyObstacles();
for k = 1:numel(vehicles)
    if ~ismember(vehicles(k).id,affected), continue; end
    prefix = fault.prefix_paths{k};
    [st_map,~] = mark_trajectory_occupancy(st_map,prefix,vehicle,vehicles(k).id);
    obstacles(end+1,1) = dynamic_vehicle_obstacle(prefix,vehicle,vehicles(k).id,planner); %#ok<AGROW>
end
st_map = prepare_dynamic_context(st_map,obstacles);

% 未受故障影響車輛保持原始優先級，在安全停车前缀的约束下先行规划。
unaffected_indices = original_order(~ismember({vehicles(original_order).id},affected));
for index = unaffected_indices
    [start_state,goal_state] = vehicleStates(vehicles(index));
    timer = tic;
    search_planner = plannerForVehicle(planner,vehicles(index).id);
    path = summon_vhybrid_astar(start_state,goal_state,geometry,vehicle,search_planner,st_map,obstacles);
    elapsed = toc(timer);
    result.timings.per_vehicle_s(index) = elapsed;
    result.timings.planning_s = result.timings.planning_s+elapsed;
    result = putPath(result,index,path,start_state,goal_state,elapsed,vehicles(index).priority);
    if ~path.valid
        result.per_vehicle(index).failure_diagnostics = path.failure_diagnostics;
        result.per_vehicle(index).reason = sprintf('search_error_%d',path.error_code);
        result.failure = summarizeSearchFailure(vehicles(index).id,path);
        return;
    end
    [st_map,~] = mark_trajectory_occupancy(st_map,path,vehicle,vehicles(index).id);
    obstacles(end+1,1) = dynamic_vehicle_obstacle(path,vehicle,vehicles(index).id,planner); %#ok<AGROW>
    st_map = prepare_dynamic_context(st_map,obstacles);
end

% 按故障后顺序构造每辆车的其他车辆地图：未处理受影响车辆仍停在其安全停车点。
for k = 1:numel(fault.replan_ids)
    id = fault.replan_ids{k};
    index = find(strcmp({vehicles.id},id),1);
    if isempty(index), error('Day7:FaultOrder','故障后顺序含未知车辆。'); end
    prefix = fault.prefix_paths{index};
    start_state = fault.stopped_states(index,:);
    [~,goal_state] = vehicleStates(vehicles(index));
    [local_map,local_obstacles] = buildOtherVehicleContext(index,vehicles, ...
        result.paths,fault.prefix_paths,affected,static_map,vehicle,planner);
    options = struct('initial_gear',prefix.gear(end));
    timer = tic;
    search_planner = plannerForVehicle(planner,id);
    suffix = summon_vhybrid_astar(start_state,goal_state,geometry,vehicle,search_planner, ...
        local_map,local_obstacles,options);
    elapsed = toc(timer);
    result.timings.per_vehicle_s(index) = elapsed;
    result.timings.planning_s = result.timings.planning_s+elapsed;
    result.per_vehicle(index).planning_time_s = elapsed;
    result.per_vehicle(index).suffix_planning_time_s = elapsed;
    if ~suffix.valid
        result.per_vehicle(index).valid = false;
        result.per_vehicle(index).error_code = double(suffix.error_code);
        result.per_vehicle(index).search_statistics = suffix.search_statistics;
        result.per_vehicle(index).failure_diagnostics = suffix.failure_diagnostics;
        result.per_vehicle(index).reason = sprintf('suffix_search_error_%d',suffix.error_code);
        result.failure = summarizeSearchFailure(vehicles(index).id,suffix);
        % 保留已验证安全的停车前缀；失败后缀不会伪装成完整轨迹。
        return;
    end
    merged = mergePrefixSuffix(prefix,suffix,goal_state,vehicle,planner,geometry);
    result = putPath(result,index,merged, ...
        [merged.x(1),merged.y(1),merged.theta(1),merged.v(1),merged.t(1)], ...
        goal_state,elapsed,result.per_vehicle(index).effective_priority);
    result.per_vehicle(index).prefix_point_count = numel(prefix.t);
    result.per_vehicle(index).suffix_planning_time_s = elapsed;
    if ~merged.valid
        result.per_vehicle(index).valid = false;
        result.per_vehicle(index).reason = 'merged_prefix_suffix_validation_failed';
        result.failure = struct('vehicle_id',vehicles(index).id, ...
            'reason','merged_prefix_suffix_validation_failed', ...
            'kinematic_validation',merged.kinematic_validation, ...
            'final_position_error_m',merged.final_position_error_m, ...
            'final_heading_error_rad',merged.final_heading_error_rad, ...
            'final_speed_error_mps',merged.final_speed_error_mps);
        return;
    end
end
ok = true;
end

function [st_map,obstacles] = buildOtherVehicleContext(ego_index,vehicles,paths, ...
    prefixes,affected,static_map,vehicle,planner)
%BUILDOTHERVEHICLECONTEXT 重建Day5占用，排除当前自车并保留其他故障车停车位。
st_map = static_map; obstacles = emptyObstacles();
for j = 1:numel(vehicles)
    if j==ego_index, continue; end
    candidate = paths{j};
    if isempty(candidate) || ~isstruct(candidate) || ~isfield(candidate,'valid') || ~candidate.valid
        if ismember(vehicles(j).id,affected), candidate = prefixes{j}; else, continue; end
    end
    [st_map,~] = mark_trajectory_occupancy(st_map,candidate,vehicle,vehicles(j).id);
    obstacles(end+1,1) = dynamic_vehicle_obstacle(candidate,vehicle,vehicles(j).id,planner); %#ok<AGROW>
end
st_map = prepare_dynamic_context(st_map,obstacles);
end

function merged = mergePrefixSuffix(prefix,suffix,goal,vehicle,planner,geometry)
%MERGEPREFIXSUFFIX 共用安全停車狀態只保留一次，並重放整條真實控制序列。
merged = suffix;
if isempty(suffix.t), return; end
sample_fields = {'x','y','theta','v','t','acceleration','steering_angle', ...
    'travelled_distance','direction','gear','is_gear_change'};
for k = 1:numel(sample_fields)
    name = sample_fields{k};
    if ~isfield(prefix,name) || ~isfield(suffix,name), continue; end
    a = prefix.(name); b = suffix.(name);
    merged.(name) = [a(:);b(2:end)];
end
merged.valid = false;
merged.goal_pose = goal(1:3);
merged.final_position_error_m = hypot(merged.x(end)-goal(1),merged.y(end)-goal(2));
merged.final_heading_error_rad = abs(wrapAngle(merged.theta(end)-goal(3)));
merged.final_speed_error_mps = abs(merged.v(end)-planner.vhybrid.terminal_speed_mps);
merged.kinematic_validation = vhybrid_validate_trajectory(merged,geometry,vehicle,planner);
merged.valid = suffix.valid && merged.kinematic_validation.valid && ...
    merged.final_position_error_m <= planner.vhybrid.goal_position_tolerance_m && ...
    merged.final_heading_error_rad <= planner.vhybrid.goal_heading_tolerance_rad && ...
    merged.final_speed_error_mps <= planner.vhybrid.goal_speed_tolerance_mps;
if merged.valid, merged.error_code = uint8(0); else, merged.error_code = uint8(8); end
merged.fault_prefix_validated = true;
end

function [pairs,all_ok] = validateAllPairs(paths,ids,geometry,vehicle,planner,resource_planner)
%VALIDATEALLPAIRS 獨立檢查所有車對的連續矩形碰撞和Day5資源塊交集。
n = numel(paths); pair_count = n*(n-1)/2;
template = struct('vehicle_a','','vehicle_b','','collision',true,'conflict_reason','not_checked', ...
    'minimum_distance_m',NaN,'minimum_distance_lower_bound_m',NaN,'conflict_time_s',NaN, ...
    'conflict_position_xy',[NaN,NaN],'resource_overlap_blocks',NaN,'valid',false);
pairs = repmat(template,pair_count,1); all_ok = true; row = 0;
horizon = resource_planner.st_occupancy.t_max-max(1e-9,eps(resource_planner.st_occupancy.t_max));
resource_planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
for i = 1:n-1
    for j = i+1:n
        row = row+1;
        a = paths{i}; b = paths{j};
        if isempty(a) || isempty(b) || ~a.valid || ~b.valid
            pairs(row) = failedPair(ids{i},ids{j},'path_missing_or_invalid');
            all_ok = false; continue;
        end
        states_a = heldStates(a,horizon);
        obstacle_b = dynamic_vehicle_obstacle(b,vehicle,ids{j},planner);
        [collision,detail] = check_trajectory_conflict(states_a,obstacle_b,vehicle,planner,[]);
        % 独立Day5写入只含该车对，避免搜索期资源表的自车占用影响结论。
        pair_map = st_occupancy_map(resource_planner,geometry);
        pair_map = mark_trajectory_occupancy(pair_map,a,vehicle,ids{i});
        [~,resource_stats] = mark_trajectory_occupancy(pair_map,b,vehicle,ids{j});
        pairs(row) = template;
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

function states = heldStates(path,horizon)
%HELDSTATES 将停车末态驻留到地图半开时间上界前一瞬。
states = [path.x(:),path.y(:),path.theta(:),path.v(:),path.t(:)];
if states(end,5) < horizon
    final = states(end,:); final(4)=0; final(5)=horizon;
    states(end+1,:) = final;
end
end

function [ok,report] = validatePoses(vehicles,geometry,vehicle,planner)
%VALIDATEPOSES 检查全部起终点车身矩形和四车初始位置的安全重叠。
n = numel(vehicles); valid_pose = true(n,2); details = cell(n,2);
for k = 1:n
    poses = [vehicles(k).start_pose.x_m,vehicles(k).start_pose.y_m,vehicles(k).start_pose.yaw_rad; ...
        vehicles(k).summon_goal.x_m,vehicles(k).summon_goal.y_m,vehicles(k).summon_goal.yaw_rad];
    for p = 1:2
        [hit,info] = day6_static_collision_check(poses(p,:),geometry,vehicle);
        valid_pose(k,p) = ~hit; details{k,p} = info;
    end
end
pair_template = struct('vehicle_a','','vehicle_b','','overlap',false,'minimum_distance_m',NaN);
initial_pairs = repmat(pair_template,n*(n-1)/2,1); row = 0;
for i=1:n-1
    for j=i+1:n
        row=row+1;
        pose_i = [vehicles(i).start_pose.x_m,vehicles(i).start_pose.y_m,vehicles(i).start_pose.yaw_rad,0,0];
        pose_j = [vehicles(j).start_pose.x_m,vehicles(j).start_pose.y_m,vehicles(j).start_pose.yaw_rad,0,0];
        [hit,dist] = check_vehicle_vehicle_collision(pose_i,pose_j,vehicle,vehicle,planner.day6);
        initial_pairs(row) = struct('vehicle_a',vehicles(i).id,'vehicle_b',vehicles(j).id, ...
            'overlap',hit,'minimum_distance_m',dist.minimum_distance_m);
    end
end
ok = all(valid_pose(:)) && ~any([initial_pairs.overlap]);
report = struct('valid',ok,'pose_valid',valid_pose,'pose_details',{details}, ...
    'initial_pairwise',initial_pairs);
end

function [ok,data,message] = validateFaultEvent(event,vehicles,geometry,vehicle,planner)
%VALIDATEFAULTEVENT 只接受给出真实控制前缀和安全静止状态的明确故障事件。
ok=false; data=struct(); message='';
required={'cause','failed_vehicle_id','affected_vehicles'};
text_cause=isstruct(event) && isscalar(event) && isfield(event,'cause') && ...
    (ischar(event.cause) && isrow(event.cause) || isstring(event.cause) && isscalar(event.cause));
text_failed=isstruct(event) && isscalar(event) && isfield(event,'failed_vehicle_id') && ...
    (ischar(event.failed_vehicle_id) && isrow(event.failed_vehicle_id) || ...
    isstring(event.failed_vehicle_id) && isscalar(event.failed_vehicle_id));
if ~isstruct(event) || ~isscalar(event) || ~all(isfield(event,required)) || ...
        ~text_cause || ~text_failed || isempty(char(string(event.cause))) || ...
        isempty(char(string(event.failed_vehicle_id)))
    message='fault_event必须包含非空cause、failed_vehicle_id和affected_vehicles。'; return;
end
affected=event.affected_vehicles;
if ~isstruct(affected) || isempty(affected) || ~all(isfield(affected,{'id','prefix_path','stopped_state'}))
    message='affected_vehicles必须包含id、prefix_path和stopped_state。'; return;
end
ids={affected.id}; scenario_ids={vehicles.id};
if numel(unique(ids))~=numel(ids) || any(~ismember(ids,scenario_ids)) || ...
        ~ismember(char(string(event.failed_vehicle_id)),ids)
    message='故障车辆和受影响车辆必须是场景内唯一ID，且failed_vehicle_id应包含在affected_vehicles中。'; return;
end
prefixes=cell(numel(vehicles),1); stopped=zeros(numel(vehicles),5);
for k=1:numel(affected)
    idx=find(strcmp(scenario_ids,ids{k}),1); path=affected(k).prefix_path;
    stop=double(affected(k).stopped_state(:).');
    control_fields={'x','y','theta','v','t','acceleration','steering_angle','gear'};
    if ~isstruct(path) || ~isscalar(path) || ~all(isfield(path,[control_fields,{'valid'}])) || ...
            ~path.valid || numel(stop)~=5 || any(~isfinite(stop)) || abs(stop(4))>1e-9
        message=sprintf('%s的故障前缀或停车状态缺少字段、无效或越出时间范围。',ids{k}); return;
    end
    try
        prefix_states=day6_trajectory_states(path);
    catch exception
        message=sprintf('%s前缀状态无效：%s',ids{k},exception.message); return;
    end
    if prefix_states(end,5)>=planner.st_occupancy.t_max
        message=sprintf('%s前缀结束时间超出时空地图范围。',ids{k}); return;
    end
    [expected_start,~]=vehicleStates(vehicles(idx));
    initial=prefix_states(1,:); final=prefix_states(end,:);
    tol=planner.vhybrid.kinematic_validation_tolerance;
    start_mismatch=max(abs(initial([1,2,4,5])-expected_start([1,2,4,5])))>tol || ...
        abs(wrapAngle(initial(3)-expected_start(3)))>tol;
    stop_mismatch=max(abs(final([1,2,4,5])-stop([1,2,4,5])))>tol || ...
        abs(wrapAngle(final(3)-stop(3)))>tol;
    if start_mismatch || stop_mismatch || ...
            abs(path.v(end))>planner.vhybrid.goal_speed_tolerance_mps
        message=sprintf('%s前缀必须从场景起点[t=0,v=0]开始并精确结束于给定安全停车状态。',ids{k}); return;
    end
    try
        replay=vhybrid_validate_trajectory(path,geometry,vehicle,planner);
    catch exception
        message=sprintf('%s前缀无法运动学重放：%s',ids{k},exception.message); return;
    end
    [stop_collision,~]=day6_static_collision_check(stop(1:3),geometry,vehicle);
    if ~replay.valid || stop_collision
        message=sprintf('%s前缀运动学/静态停车安全检查失败。',ids{k}); return;
    end
    prefixes{idx}=path; stopped(idx,:)=stop;
end
affected_indices=ismember(scenario_ids,ids);
data=struct('cause',char(string(event.cause)), ...
    'failed_vehicle_id',char(string(event.failed_vehicle_id)),'affected_ids',{ids}, ...
    'prefix_paths',{prefixes},'stopped_states',stopped, ...
    'stop_times',stopped(affected_indices,5));
ok=true;
end

function result = putPath(result,index,path,start_state,goal_state,elapsed,priority)
%PUTPATH 保存路径摘要及真实搜索统计。
result.paths{index}=path;
record=result.per_vehicle(index);
record.valid=path.valid; record.error_code=double(path.error_code);
record.planning_time_s=elapsed; record.search_statistics=path.search_statistics;
record.failure_diagnostics=path.failure_diagnostics; record.start_state=start_state;
record.goal_state=goal_state; record.priority=priority;
record.final_state=zeros(1,5);
if ~isempty(path.t), record.final_state=[path.x(end),path.y(end),path.theta(end),path.v(end),path.t(end)]; end
record.starts_at_rest_t0=~isempty(path.t) && abs(path.v(1))<1e-9 && abs(path.t(1))<1e-9;
record.moves_immediately=~isempty(path.t) && numel(path.t)>1 && path.v(2)>1e-9 && ...
    path.acceleration(2)>0 && hypot(path.x(2)-path.x(1),path.y(2)-path.y(1))>1e-10;
record.terminal_speed_zero=~isempty(path.v) && abs(path.v(end))<=1e-8;
record.kinematic_valid=isfield(path,'kinematic_validation') && ...
    isfield(path.kinematic_validation,'valid') && path.kinematic_validation.valid;
record.reason='';
result.per_vehicle(index)=record;
end

function record = vehicleRecord(vehicle)
%VEHICLERECORD 创建可写JSON的车辆摘要。
record=struct('id','','priority',0,'effective_priority',0,'valid',false,'error_code',NaN, ...
    'reason','not_planned','planning_time_s',0,'suffix_planning_time_s',NaN, ...
    'prefix_point_count',0,'start_state',zeros(1,5),'goal_state',zeros(1,5), ...
    'final_state',zeros(1,5),'starts_at_rest_t0',false,'moves_immediately',false, ...
    'terminal_speed_zero',false,'kinematic_valid',false,'search_statistics',struct(), ...
    'failure_diagnostics',struct());
if nargin>0
    record.id=vehicle.id; record.priority=vehicle.priority; record.effective_priority=vehicle.priority;
end
end

function [start,goal]=vehicleStates(vehicle)
%VEHICLESTATES Day7统一从t=0、v=0构造状态。
start=[vehicle.start_pose.x_m,vehicle.start_pose.y_m,vehicle.start_pose.yaw_rad,0,0];
goal=[vehicle.summon_goal.x_m,vehicle.summon_goal.y_m,vehicle.summon_goal.yaw_rad,0,0];
end

function [vehicles,order]=sortVehicles(vehicles)
%SORTVEHICLES 依显式优先级升序规划，保留输入数组作稳定平局顺序。
priorities=double([vehicles.priority]);
[~,order]=sortrows([priorities(:),(1:numel(priorities)).'],[1,2]);
vehicles=vehicles(order);
order=1:numel(vehicles);
end

function validateScenario(scenario)
%VALIDATESCENARIO 检查四车场景和优先级字段。
if ~isstruct(scenario) || ~isfield(scenario,'vehicles') || numel(scenario.vehicles)~=4
    error('Day7:InvalidScenario','Day7场景必须包含恰好四辆车。');
end
v=scenario.vehicles;
if ~all(isfield(v,{'id','priority','start_pose','summon_goal'})) || ...
        numel(unique({v.id}))~=4 || numel(unique([v.priority]))~=4
    error('Day7:InvalidScenario','车辆ID、优先级或位姿字段无效。');
end
for k=1:numel(v)
    if ~all(isfield(v(k).start_pose,{'x_m','y_m','yaw_rad'})) || ...
            ~all(isfield(v(k).summon_goal,{'x_m','y_m','yaw_rad'})) || ...
            any(~isfinite([v(k).start_pose.x_m,v(k).start_pose.y_m,v(k).start_pose.yaw_rad, ...
                v(k).summon_goal.x_m,v(k).summon_goal.y_m,v(k).summon_goal.yaw_rad]))
        error('Day7:InvalidScenario','车辆位姿字段必须有限且完整。');
    end
end
end

function obstacles=emptyObstacles()
%EMPTYOBSTACLES 创建Day6障碍结构的空数组。
obstacles=struct('id',{},'states',{},'vehicle_config',{},'footprint',{},'hold_end',{});
end

function pair=failedPair(id_a,id_b,reason)
%FAILEDPAIR 表示因路径缺失未能执行的车对验收。
pair=struct('vehicle_a',id_a,'vehicle_b',id_b,'collision',true, ...
    'conflict_reason',reason,'minimum_distance_m',NaN, ...
    'minimum_distance_lower_bound_m',NaN,'conflict_time_s',NaN, ...
    'conflict_position_xy',[NaN,NaN],'resource_overlap_blocks',NaN,'valid',false);
end

function angle=wrapAngle(angle)
%WRAPANGLE 航向误差归一化到[-pi,pi)。
angle=mod(angle+pi,2*pi)-pi;
end

function result=finishDay7(result,enabled,output_dir,total_timer,geometry,vehicle)
%FINISHDAY7 写Day7报告/状态表，仅在全车验收通过时写同步动画和完整路径表。
if ~isfield(result,'timings') || isempty(result.timings)
    result.timings=struct('per_vehicle_s',zeros(0,1),'planning_s',0, ...
        'pairwise_validation_s',0,'plot_s',0,'animation_s',0,'table_output_s',0, ...
        'file_output_s',0,'total_runtime_s',0);
end
if enabled
    if ~exist(output_dir,'dir'), mkdir(output_dir); end
    % 先清除旧图表，防止本次绘图失败后将上一轮成功结果误认为当前结果。
    stale_names={'day7_four_vehicle_trajectories.png','day7_four_vehicle_speed_time.png', ...
        'day7_four_vehicle_animation.gif','day7_animation_preview.png', ...
        'day7_vehicle_timing.csv','day7_trajectory_samples.csv'};
    for stale_index=1:numel(stale_names)
        stale_file=fullfile(output_dir,stale_names{stale_index});
        if exist(stale_file,'file'), delete(stale_file); end
    end
    plot_files=struct('xy_plot',fullfile(output_dir,'day7_four_vehicle_trajectories.png'), ...
        'speed_plot',fullfile(output_dir,'day7_four_vehicle_speed_time.png'));
    plot_timer=tic;
    try
        makeDay7Plots(result,geometry,vehicle,plot_files);
    catch exception
        result.output_files.plot_error=exception.message;
    end
    result.timings.plot_s=toc(plot_timer);
    plot_names=fieldnames(plot_files);
    for plot_index=1:numel(plot_names)
        plot_name=plot_names{plot_index};
        if exist(plot_files.(plot_name),'file')
            result.output_files.(plot_name)=plot_files.(plot_name);
        end
    end
    complete_paths = iscell(result.paths) && numel(result.paths)==4 && numel(result.per_vehicle)==4 && ...
        all([result.per_vehicle.valid]) && all(cellfun(@(p)isstruct(p) && ...
        isfield(p,'valid') && p.valid && isfield(p,'t') && ~isempty(p.t),result.paths));
    write_complete_outputs = result.valid && complete_paths;
    if ~write_complete_outputs
        result.output_files.final_outputs_skipped= ...
            '联合规划或四条完整路径未通过验收；保留逐车状态/耗时表，未生成同步动画和完整路径CSV。';
    end
    try
        output_planner=struct();
        if write_complete_outputs
            output_planner=jsondecode(fileread(fullfile( ...
                fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
                'config','planner_config_day7.json')));
        end
        [output_files,output_times] = day7_write_animation_and_tables( ...
            result.paths,result.per_vehicle,geometry,vehicle,output_planner, ...
            output_dir,write_complete_outputs);
        names = fieldnames(output_files);
        for output_index=1:numel(names)
            result.output_files.(names{output_index})=output_files.(names{output_index});
        end
        result.timings.animation_s=output_times.animation_s;
        result.timings.table_output_s=output_times.table_output_s;
    catch exception
        result.output_files.animation_table_error=exception.message;
        partial_names={'day7_four_vehicle_animation.gif','day7_animation_preview.png', ...
            'day7_trajectory_samples.csv'};
        for partial_index=1:numel(partial_names)
            partial_file=fullfile(output_dir,partial_names{partial_index});
            if exist(partial_file,'file'), delete(partial_file); end
        end
        timing_file=fullfile(output_dir,'day7_vehicle_timing.csv');
        if exist(timing_file,'file')
            result.output_files.vehicle_timing_csv=timing_file;
        end
    end
    result.output_files.report_json=fullfile(output_dir,'day7_four_vehicle_report.json');
    result.output_files.trajectory_mat=fullfile(output_dir,'day7_four_vehicle_trajectories.mat');
    output_timer=tic;
    mat_written=false;
    try
        % 主数据paths只保存一次，省去巨大的逐扩展节点云数组。
        % result元数据在计时结束后单独写入，与JSON使用同一份报告和timings。
        % 原始result仍完整返回调用方；MAT顶层变量paths保存完整采样路径。
        compact=compactResultForMat(result);
        mat_payload=struct('paths',{compact.paths});
        save(result.output_files.trajectory_mat,'-struct','mat_payload','-v7');
        mat_written=true;
    catch exception
        result.output_files.mat_error=exception.message;
    end
    result.timings.file_output_s=toc(output_timer);
    result.timings.total_runtime_s=toc(total_timer);
    if mat_written
        try
            report=compactReport(result);
            metadata=struct('result',report,'timings',result.timings);
            save(result.output_files.trajectory_mat,'-struct','metadata','-append');
        catch exception
            result.output_files.mat_error=exception.message;
        end
    end
    % JSON独立写出；即使MAT失败，当前运行及其保存错误仍写入本次报告。
    try
        report=compactReport(result);
        fid=fopen(result.output_files.report_json,'w','n','UTF-8');
        if fid<0, error('Day7:OutputFailure','无法创建Day7报告。'); end
        cleanup=onCleanup(@() fclose(fid));
        fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
        clear cleanup;
    catch exception
        result.output_files.json_error=exception.message;
    end
end
fprintf('Day7 result=%s; planning %.3f s, pair validation %.3f s.\n', ...
    result.reason,fieldOrZero(result.timings,'planning_s'), ...
    fieldOrZero(result.timings,'pairwise_validation_s'));
end

function compact=compactResultForMat(result)
%COMPACTRESULTFORMAT 仅为独立MAT输出移除可由搜索统计替代的扩展节点云。
compact=result;
for k=1:numel(compact.paths)
    path=compact.paths{k};
    if isstruct(path) && isfield(path,'search_nodes')
        path.search_nodes=repmat(vhybrid_node(),0,1);
        compact.paths{k}=path;
    end
end
end

function makeDay7Plots(result,geometry,vehicle,files)
%MAKEDAY7PLOTS 生成轨迹/起终点图和带方向速度时间图，失败时保留全车位姿。
colors=lines(max(4,numel(result.paths)));
f=figure('Visible','off','Color','w'); cleanup=onCleanup(@() close(f));
hold on; axis equal; grid on;
boundary=geometry.boundary_xy;
plot(boundary(:,1),boundary(:,2),'k-','LineWidth',1.5);
labels=cell(numel(result.paths),1);
xy_handles=gobjects(numel(result.paths),1);
for k=1:numel(result.paths)
    labels{k}=result.per_vehicle(k).id;
    start_state=result.per_vehicle(k).start_state;
    goal_state=result.per_vehicle(k).goal_state;
    plot(start_state(1),start_state(2),'o','Color',colors(k,:), ...
        'MarkerFaceColor',colors(k,:),'HandleVisibility','off');
    plot(goal_state(1),goal_state(2),'x','Color',colors(k,:), ...
        'LineWidth',1.5,'MarkerSize',8,'HandleVisibility','off');
    path=result.paths{k};
    if isempty(path) || ~isstruct(path) || ~isfield(path,'t') || isempty(path.t)
        xy_handles(k)=plot(nan,nan,'-','Color',colors(k,:),'LineWidth',1.7);
        continue;
    end
    if path.valid
        xy_handles(k)=plot(path.x,path.y,'-','Color',colors(k,:),'LineWidth',1.7);
        drawFootprint(path,vehicle,colors(k,:));
    else
        xy_handles(k)=plot(nan,nan,'-','Color',colors(k,:),'LineWidth',1.7);
    end
end
xlim([0,30]); ylim([0,30]); xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Day 7 four-vehicle planning: %s',strrep(result.reason,'_',' ')), ...
    'Interpreter','none');
legend(xy_handles,labels,'Location','bestoutside','Interpreter','none');
if isfield(result,'failure') && isfield(result.failure,'conflict_position_xy') && ...
        all(isfinite(result.failure.conflict_position_xy))
    plot(result.failure.conflict_position_xy(1),result.failure.conflict_position_xy(2), ...
        'mx','LineWidth',2,'MarkerSize',11,'HandleVisibility','off');
end
exportgraphics(f,files.xy_plot,'Resolution',150); clear cleanup;

f=figure('Visible','off','Color','w'); cleanup=onCleanup(@() close(f));
hold on; grid on;
speed_handles=gobjects(numel(result.paths),1);
for k=1:numel(result.paths)
    labels{k}=result.per_vehicle(k).id;
    path=result.paths{k};
    if isempty(path) || ~isstruct(path) || ~isfield(path,'t') || isempty(path.t)
        speed_handles(k)=plot(nan,nan,'Color',colors(k,:),'LineWidth',1.5);
        continue;
    end
    signed_speed=path.v(:);
    if isfield(path,'gear'), signed_speed=signed_speed.*path.gear(:); end
    speed_handles(k)=plot(path.t,signed_speed,'Color',colors(k,:),'LineWidth',1.5);
end
xlabel('time (s)'); ylabel('signed speed (m/s)'); title('Day 7 speed-time profiles');
legend(speed_handles,labels,'Location','best','Interpreter','none');
exportgraphics(f,files.speed_plot,'Resolution',150); clear cleanup;
end

function drawFootprint(path,vehicle,color)
%DRAWFOOTPRINT 在轨迹首末端画含安全裕度的实际朝向矩形。
indices=unique([1,numel(path.t)]);
footprint=inflate_vehicle_occupancy(vehicle,0);
for idx=indices
    theta=path.theta(idx); c=cos(theta); s=sin(theta);
    corners=footprint.corners_local*[c,s;-s,c]+[path.x(idx),path.y(idx)];
    closed_indices=[1:size(corners,1),1];
    plot(corners(closed_indices,1),corners(closed_indices,2), ...
        '-','Color',color,'LineWidth',0.9);
end
end

function report=compactReport(result)
%COMPACTREPORT JSON包含错误、逐车搜索统计、所有车对和计时。
report=rmfield(result,'paths');
end

function value=fieldOrZero(s,name)
%FIELDORZERO 失败发生在配置之前时输出安全计时默认值。
if isfield(s,name), value=s.(name); else, value=0; end
end

function value=geometryOrEmpty()
%GEOMETRYOREMPTY 异常早于地图加载时供输出过程使用。
value=struct('boundary_xy',[0,0;30,0;30,30;0,30;0,0],'obstacles',{{}});
end

function value=vehicleOrEmpty()
%VEHICLEOREMPTY 生成绘图所需默认空车型。
value=struct('dimensions_m',struct('width',1,'wheelbase',1, ...
    'front_axle_to_front',0,'rear_axle_to_rear',0),'collision_margin_m', ...
    struct('front',0,'rear',0,'side',0));
end

