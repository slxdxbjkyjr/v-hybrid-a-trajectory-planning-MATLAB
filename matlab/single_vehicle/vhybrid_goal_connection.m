function [success, connection_nodes, statistics] = vhybrid_goal_connection( ...
    parent_node, goal_state, map, vehicle_config, planner_config, st_map, higher_priority_trajectories)
%VHYBRID_GOAL_CONNECTION 用真实 Dubins 控制段和既有运动学连接终点。
% 输入：父节点 [x,y,theta,v,t]、目标状态、静态地图和车辆/规划配置。
% 输出：success、父节点之后的连接节点、候选/碰撞/约束裁剪统计。
% 逻辑：枚举六类圆弧/直线组合，按最小转弯半径限制曲率；在弧长上
% 构造加速/匀速/减速剖面。每个点都由 summon_vehicle_dynamic 积分，
% 在几何段边界与速度相位边界切步。禁止直接赋值目标位置或航向。

cfg = planner_config.vhybrid;
dyn = planner_config.dynamics;
lateral_cfg = validate_lateral_acceleration_config(dyn);
% 缺省仍接受首条可行连接；Day7可按同一父节点的完整累计代价择优。
selection_mode = 'first_feasible';
if isfield(cfg,'goal_connection_selection')
    selection_mode = cfg.goal_connection_selection;
    if ~((ischar(selection_mode) && isrow(selection_mode)) || ...
            (isstring(selection_mode) && isscalar(selection_mode))) || ...
            ~any(strcmp(selection_mode,{'first_feasible','lowest_cost'}))
        error('VHybrid:InvalidGoalConnectionSelection', ...
            'goal_connection_selection 必须为 first_feasible 或 lowest_cost。');
    end
end
select_lowest_cost = strcmp(selection_mode,'lowest_cost');
best_connection_cost = Inf;
% 缺省不限制连接累计转角，保留Day6；有限门限可拒绝近目标整圈绕行。
max_connection_turn = Inf;
if isfield(cfg,'goal_connection_max_turn_rad')
    max_connection_turn = cfg.goal_connection_max_turn_rad;
    if ~isnumeric(max_connection_turn) || ~isreal(max_connection_turn) || ...
            ~isscalar(max_connection_turn) || isnan(max_connection_turn) || ...
            max_connection_turn <= 0
        error('VHybrid:InvalidGoalConnectionMaxTurn', ...
            'goal_connection_max_turn_rad 必须为正有限实数标量或 Inf。');
    end
end
% Day7 可选多半径连接；缺省仅使用原最小半径，保留 Day6 行为。
radius_scales = 1;
if isfield(cfg,'goal_connection_radius_scales')
    radius_scales = cfg.goal_connection_radius_scales;
    if ~isnumeric(radius_scales) || ~isreal(radius_scales) || ...
            isempty(radius_scales) || ~isvector(radius_scales) || ...
            any(~isfinite(radius_scales(:))) || any(radius_scales(:)<1)
        error('VHybrid:InvalidGoalConnectionRadiusScales', ...
            'goal_connection_radius_scales 必须为有限且不小于1的非空实数向量。');
    end
    radius_scales = unique(double(radius_scales(:).'),'stable');
end
% 可选方案C仅由显式配置开启，缺省保持Day6的速度剖面行为。
multi_phase_enabled = false;
if isfield(cfg,'goal_connection_multi_phase_profile')
    multi_phase_enabled = cfg.goal_connection_multi_phase_profile;
    if ~islogical(multi_phase_enabled) || ~isscalar(multi_phase_enabled)
        error('VHybrid:InvalidMultiPhaseProfile', ...
            'goal_connection_multi_phase_profile 必须为逻辑标量。');
    end
end
if nargin < 6, st_map = []; end
if nargin < 7, higher_priority_trajectories = []; end
has_context = ~isempty(st_map) || ~isempty(higher_priority_trajectories);
success = false;
connection_nodes = repmat(vhybrid_node(), 0, 1);
statistics = struct('candidates', 0, 'collision_pruned', 0, ...
    'constraint_pruned', 0, 'terminal_speed', NaN, 'connection_length_m', NaN, ...
    'lateral_acceleration_pruned',0, ...
    'dynamic_pruned',0,'resource_pruned',0,'reason_counts',struct(), ...
    'conflict_examples',{{}},'geometric_candidates',0,'boundary_geometry_pruned',0, ...
    'speed_profile_diagnostics',emptySpeedProfileDiagnostics(lateral_cfg.enabled), ...
    'turning_geometry_pruned',0, ...
    'cost_bound_enabled',false,'cost_bound_pruned',0, ...
    'cost_bound_integration_pruned',0,'multi_phase_candidates',0, ...
    'multi_phase_collision_pruned',0,'multi_phase_cost_pruned',0, ...
    'multi_phase_selected',false,'multi_phase_stop_wait_s',NaN, ...
    'multi_phase_cruise_speed_mps',NaN,'multi_phase_stop_distance_fraction',NaN);
start = [parent_node.x,parent_node.y,parent_node.theta,parent_node.v,parent_node.t];
gear = 1;
if isfield(parent_node,'gear'), gear = parent_node.gear; end
% 同挡连接不含换挡项；只在实际累积代价的每一项均非负时剪枝。
% 可关闭剪枝以与完整枚举作精确回归比较，first_feasible始终不启用。
pruning_requested = true;
if select_lowest_cost && isfield(cfg,'goal_connection_cost_pruning_enabled')
    pruning_requested = cfg.goal_connection_cost_pruning_enabled;
    if ~islogical(pruning_requested) || ~isscalar(pruning_requested)
        error('VHybrid:InvalidGoalConnectionCostPruning', ...
            'goal_connection_cost_pruning_enabled 必须为逻辑标量。');
    end
end
statistics.cost_bound_enabled = select_lowest_cost && pruning_requested && ...
    nonnegativeConnectionCosts(cfg,gear,parent_node.g_cost);
if gear < 0 && isfield(cfg,'max_reverse_speed_mps')
    dyn.v_max_mps = min(dyn.v_max_mps,cfg.max_reverse_speed_mps);
end
if multi_phase_enabled
    multi_phase_config = readMultiPhaseConfig(cfg,dyn);
end
goal_state = double(goal_state(:).');
if hypot(start(1)-goal_state(1),start(2)-goal_state(2)) > cfg.goal_connection_distance_m
    return;
end
% 连接过程冻结静态几何；只在入口检查缓存源，逐积分点使用可信内核。
map = prepare_static_context(map,vehicle_config);
delta_max = min(vehicle_config.steering.maximum_steer_rad, dyn.delta_max_rad);
radius = vehicle_config.dimensions_m.wheelbase / tan(delta_max);
% 倒车的运动切向比车身航向多 pi；同挡位连接不隐式换挡。
motion_start = start(1:3); motion_goal = goal_state(1:3);
if gear<0
    motion_start(3) = motion_start(3)+pi; motion_goal(3) = motion_goal(3)+pi;
end
curves = repmat(struct('lengths',zeros(1,3),'types',zeros(1,3),'radius_m',radius),0,1);
for radius_scale = radius_scales
    actual_radius = radius*radius_scale;
    if ~isfinite(actual_radius)
        error('VHybrid:InvalidGoalConnectionRadiusScales', ...
            'goal_connection_radius_scales 产生了非有限转弯半径。');
    end
    radius_curves = vhybrid_dubins_candidates(motion_start,motion_goal,actual_radius);
    if isempty(radius_curves), continue; end
    for curve_id = 1:numel(radius_curves)
        radius_curves(curve_id).radius_m = actual_radius;
    end
    curves = [curves;radius_curves]; %#ok<AGROW>
end
% 不同半径的候选统一按几何总弧长排序；同长时保持既有候选次序。
if ~isempty(curves)
    [~,curve_order] = sort(arrayfun(@(candidate)sum(candidate.lengths),curves));
    curves = curves(curve_order);
end
% 采样的是连接段巡航速度，终端速度始终为目标停车约束。
speeds = unique([cfg.goal_connection_speed_samples_mps(:).', ...
    cfg.reference_speed_mps, start(4), cfg.terminal_speed_mps]);
speeds = sort(speeds, 'descend');
for curve_id = 1:numel(curves)
    curve = curves(curve_id);
    statistics.geometric_candidates = statistics.geometric_candidates+1;
    % L/R段弧长除以该候选自身半径得到绝对转角；直线段不计入。
    % 这是几何筛选，不通过换速度或积分采样绕过门限。
    cumulative_turn = sum(curve.lengths(curve.types~=0))/curve.radius_m;
    if cumulative_turn > max_connection_turn
        statistics.turning_geometry_pruned = statistics.turning_geometry_pruned+1;
        key = 'excessive_connection_turn';
        if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
        statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
        continue;
    end
    % 先用车身角点完整圆弧极值证明是否必定越界，再尝试速度相位。
    % 这避免反复用不同速度积分同一条必不可行的几何曲线。
    [outside,~] = vhybrid_curve_boundary_check(start,curve,curve.radius_m,map,vehicle_config,gear);
    if outside
        statistics.boundary_geometry_pruned = statistics.boundary_geometry_pruned+1;
        key = 'analytic_boundary_collision';
        if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
        statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
        continue;
    end
    for cruise_speed = speeds
        statistics.candidates = statistics.candidates + 1;
        if lateral_cfg.enabled
            segment_curvatures = abs(curve.types)/curve.radius_m;
            [phases,feasible,profile_diagnostic] = vhybrid_segmented_speed_phases( ...
                curve.lengths,segment_curvatures,start(4),cfg.terminal_speed_mps, ...
                cruise_speed,dyn,lateral_cfg.max_lateral_acceleration_mps2, ...
                lateral_cfg.lateral_acceleration_tolerance_mps2);
            profile_diagnostic.candidate_kind = 'ordinary';
            statistics.speed_profile_diagnostics = recordSpeedProfile( ...
                statistics.speed_profile_diagnostics,profile_diagnostic);
        else
            [phases, feasible] = vhybrid_speed_phases(sum(curve.lengths),start(4), ...
                cfg.terminal_speed_mps,cruise_speed,dyn);
            profile_diagnostic = struct();
        end
        if ~feasible
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            if lateral_cfg.enabled && ...
                    profile_diagnostic.lateral_constraint_caused_infeasibility
                statistics.lateral_acceleration_pruned = ...
                    statistics.lateral_acceleration_pruned+1;
                statistics = countReason(statistics,profile_diagnostic.reason);
            end
            continue;
        end
        cost_limit = Inf;
        if statistics.cost_bound_enabled && isfinite(best_connection_cost)
            cost_limit = best_connection_cost;
        end
        actual_delta = delta_max;
        if curve.radius_m ~= radius
            actual_delta = atan(vehicle_config.dimensions_m.wheelbase/curve.radius_m);
        end
        [trial, reason] = integrateCurve(parent_node, goal_state, curve, phases, ...
            actual_delta, map, vehicle_config, planner_config, cost_limit);
        if strcmp(reason,'collision')
            statistics.collision_pruned = statistics.collision_pruned + 1;
            % 静态碰撞仅依赖几何，对同一曲线换速度无法绕开障碍物。
            break;
        elseif strcmp(reason,'cost_bound')
            statistics.cost_bound_pruned = statistics.cost_bound_pruned+1;
            statistics.cost_bound_integration_pruned = statistics.cost_bound_integration_pruned+1;
            continue;
        elseif isLateralLimitReason(reason)
            statistics.constraint_pruned = statistics.constraint_pruned+1;
            statistics.lateral_acceleration_pruned = ...
                statistics.lateral_acceleration_pruned+1;
            statistics = countReason(statistics,reason);
            continue;
        elseif ~strcmp(reason,'ok')
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            continue;
        end
        if has_context
            % 终点解析连接与普通扩展接受同一个连续碰撞验收，不能绕过
            % 资源块。动态冲突只拒绝当前速度，相同曲线仍可换速度重试。
            trajectory = connectionTrajectory(parent_node,trial);
            [collision,report] = check_trajectory_conflict(trajectory, ...
                higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                struct('collect_distance_samples',false));
            if ~collision && ~isempty(higher_priority_trajectories) && ...
                    isfield(planner_config,'day6') && planner_config.day6.goal_hold_enabled
                hold_trajectory = goalHoldTrajectory(trajectory,st_map,planner_config);
                [collision,report] = check_trajectory_conflict(hold_trajectory, ...
                    higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                    struct('collect_distance_samples',false));
            end
            if collision
                statistics.collision_pruned = statistics.collision_pruned + 1;
                statistics.dynamic_pruned = statistics.dynamic_pruned + double( ...
                    report.rectangle_collision || report.physical_collision || ...
                    (report.resource_conflict && ~strcmp(report.reason,'outside_time_space_bounds')));
                statistics.resource_pruned = statistics.resource_pruned + double(report.resource_conflict);
                key = matlab.lang.makeValidName(char(report.reason));
                if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
                statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
                if numel(statistics.conflict_examples) < planner_config.day6.max_conflict_examples
                    statistics.conflict_examples{end+1} = report;
                end
                continue;
            end
        end
        if ~select_lowest_cost || trial(end).g_cost < best_connection_cost
            connection_nodes = trial;
            statistics.terminal_speed = trial(end).v;
            statistics.connection_length_m = sum(curve.lengths);
            best_connection_cost = trial(end).g_cost;
            if lateral_cfg.enabled
                statistics.speed_profile_diagnostics.selected = profile_diagnostic;
            end
            success = true;
        end
        if ~select_lowest_cost, return; end
    end
end

for curve_id = 1:numel(curves)
    curve = curves(curve_id);
    if ~multi_phase_enabled, continue; end
    cumulative_turn=sum(curve.lengths(curve.types~=0))/curve.radius_m;
    if cumulative_turn>max_connection_turn
        continue;
    end
    [outside,~]=vhybrid_curve_boundary_check(start,curve,curve.radius_m,map,vehicle_config,gear);
    if outside
        continue;
    end
    [multi_candidates,multi_profile_attempts] = vhybrid_multi_phase_profiles(sum(curve.lengths),start(4), ...
        cfg.terminal_speed_mps,multi_phase_config.cruise_speeds_mps, ...
        multi_phase_config.stop_distance_fractions,multi_phase_config.wait_times_s,dyn, ...
        curve.lengths,abs(curve.types)/curve.radius_m,lateral_cfg.enabled, ...
        lateral_cfg.max_lateral_acceleration_mps2, ...
        lateral_cfg.lateral_acceleration_tolerance_mps2);
    for profile_id = 1:numel(multi_profile_attempts)
        profile_attempt = multi_profile_attempts{profile_id};
        statistics.speed_profile_diagnostics = recordSpeedProfile( ...
            statistics.speed_profile_diagnostics,profile_attempt);
        if ~profile_attempt.feasible
            statistics.constraint_pruned = statistics.constraint_pruned+1;
            if profile_attempt.lateral_constraint_caused_infeasibility
                statistics.lateral_acceleration_pruned = ...
                    statistics.lateral_acceleration_pruned+1;
                statistics = countReason(statistics,profile_attempt.reason);
            end
        end
    end
    statistics.multi_phase_candidates = statistics.multi_phase_candidates+numel(multi_candidates);
    for multi_id = 1:numel(multi_candidates)
        candidate = multi_candidates(multi_id);
        statistics.candidates = statistics.candidates+1;
        cost_limit = Inf;
        if statistics.cost_bound_enabled && isfinite(best_connection_cost)
            cost_limit = best_connection_cost;
        end
        actual_delta = delta_max;
        if curve.radius_m ~= radius
            actual_delta = atan(vehicle_config.dimensions_m.wheelbase/curve.radius_m);
        end
        [trial,reason] = integrateCurve(parent_node,goal_state,curve,candidate.phases, ...
            actual_delta,map,vehicle_config,planner_config,cost_limit,true);
        if strcmp(reason,'collision')
            statistics.collision_pruned=statistics.collision_pruned+1;
            statistics.multi_phase_collision_pruned=statistics.multi_phase_collision_pruned+1;
            continue;
        elseif strcmp(reason,'cost_bound')
            statistics.cost_bound_pruned=statistics.cost_bound_pruned+1;
            statistics.cost_bound_integration_pruned=statistics.cost_bound_integration_pruned+1;
            statistics.multi_phase_cost_pruned=statistics.multi_phase_cost_pruned+1;
            continue;
        elseif isLateralLimitReason(reason)
            statistics.constraint_pruned=statistics.constraint_pruned+1;
            statistics.lateral_acceleration_pruned= ...
                statistics.lateral_acceleration_pruned+1;
            statistics=countReason(statistics,reason);
            continue;
        elseif ~strcmp(reason,'ok')
            statistics.constraint_pruned=statistics.constraint_pruned+1;
            continue;
        end
        if has_context
            trajectory=connectionTrajectory(parent_node,trial);
            [collision,report]=check_trajectory_conflict(trajectory, ...
                higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                struct('collect_distance_samples',false));
            if ~collision && ~isempty(higher_priority_trajectories) && ...
                    isfield(planner_config,'day6') && planner_config.day6.goal_hold_enabled
                hold_trajectory=goalHoldTrajectory(trajectory,st_map,planner_config);
                [collision,report]=check_trajectory_conflict(hold_trajectory, ...
                    higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                    struct('collect_distance_samples',false));
            end
            if collision
                statistics.collision_pruned=statistics.collision_pruned+1;
                statistics.multi_phase_collision_pruned=statistics.multi_phase_collision_pruned+1;
                statistics.dynamic_pruned=statistics.dynamic_pruned+double( ...
                    report.rectangle_collision || report.physical_collision || ...
                    (report.resource_conflict && ~strcmp(report.reason,'outside_time_space_bounds')));
                statistics.resource_pruned=statistics.resource_pruned+double(report.resource_conflict);
                key=matlab.lang.makeValidName(char(report.reason));
                if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key)=0; end
                statistics.reason_counts.(key)=statistics.reason_counts.(key)+1;
                if numel(statistics.conflict_examples)<planner_config.day6.max_conflict_examples
                    statistics.conflict_examples{end+1}=report;
                end
                continue;
            end
        end
        if ~select_lowest_cost || trial(end).g_cost<best_connection_cost
            connection_nodes=trial;
            statistics.terminal_speed=trial(end).v;
            statistics.connection_length_m=sum(curve.lengths);
            best_connection_cost=trial(end).g_cost;
            if lateral_cfg.enabled
                statistics.speed_profile_diagnostics.selected = candidate.profile_diagnostics;
            end
            statistics.multi_phase_selected=true;
            statistics.multi_phase_stop_wait_s=candidate.wait_time_s;
            statistics.multi_phase_cruise_speed_mps=candidate.cruise_speed_mps;
            statistics.multi_phase_stop_distance_fraction=candidate.stop_distance_fraction;
            success=true;
        end
        if ~select_lowest_cost, return; end
    end
end

function trajectory = connectionTrajectory(parent,nodes)
%CONNECTIONTRAJECTORY 将真实模型积分的连接节点转换为统一带控制轨迹。
trajectory = struct('x',[parent.x;vertcat(nodes.x)], ...
    'y',[parent.y;vertcat(nodes.y)],'theta',[parent.theta;vertcat(nodes.theta)], ...
    'v',[parent.v;vertcat(nodes.v)],'t',[parent.t;vertcat(nodes.t)], ...
    'gear',[parent.gear;vertcat(nodes.gear)], ...
    'is_gear_change',[false;vertcat(nodes.is_gear_change)], ...
    'acceleration',[0;vertcat(nodes.acceleration)], ...
    'steering_angle',[0;vertcat(nodes.steering_angle)]);
end

function trajectory = goalHoldTrajectory(path,st_map,planner)
%GOALHOLDTRAJECTORY 检查终点停车后直到地图末时间层的资源占用。
if isempty(st_map), horizon = planner.st_occupancy.t_max;
else, horizon = st_map.t_max; end
last_time = horizon-max(1e-9,16*eps(horizon));
trajectory = struct('x',path.x(end),'y',path.y(end),'theta',path.theta(end), ...
    'v',path.v(end),'t',path.t(end),'acceleration',0,'steering_angle',0, ...
    'gear',path.gear(end),'is_gear_change',false);
if last_time > path.t(end)
    trajectory.x(2,1) = path.x(end); trajectory.y(2,1) = path.y(end);
    trajectory.theta(2,1) = path.theta(end); trajectory.v(2,1) = 0;
    trajectory.t(2,1) = last_time;
    trajectory.acceleration(2,1) = 0; trajectory.steering_angle(2,1) = 0;
    trajectory.gear(2,1) = path.gear(end); trajectory.is_gear_change(2,1) = false;
end
end
end

function flag = isLateralLimitReason(reason)
%ISLATERALLIMITREASON 只分类由横向加速度限值触发的运动学拒绝。
flag = any(strcmp(reason,{'entry_lateral_acceleration_exceeded', ...
    'segment_lateral_acceleration_exceeded'}));
end

function diagnostic = emptySpeedProfileDiagnostics(enabled)
%EMPTYSPEEDPROFILEDIAGNOSTICS 初始化局部限速与可达性统计。
diagnostic = struct('enabled',enabled,'attempt_count',0,'feasible_count',0, ...
    'unreachable_count',0,'reason_counts',struct(),'last_attempt',struct(), ...
    'last_unreachable',struct(),'selected',struct());
end

function summary = recordSpeedProfile(summary,diagnostic)
%RECORDSPEEDPROFILE 汇总剖面可达性并保留最近失败诊断。
summary.attempt_count = summary.attempt_count+1;
summary.last_attempt = diagnostic;
if diagnostic.feasible
    summary.feasible_count = summary.feasible_count+1;
    return;
end
summary.unreachable_count = summary.unreachable_count+1;
summary.last_unreachable = diagnostic;
key = matlab.lang.makeValidName(char(diagnostic.reason));
if ~isfield(summary.reason_counts,key), summary.reason_counts.(key) = 0; end
summary.reason_counts.(key) = summary.reason_counts.(key)+1;
end

function statistics = countReason(statistics,reason)
%COUNTREASON 汇总独立拒绝原因。
key = matlab.lang.makeValidName(char(reason));
if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
end


function [nodes, reason] = integrateCurve(parent, goal, curve, phases, ...
    actual_delta, map, vehicle, planner, cost_limit, allow_stationary_wait)
%INTEGRATECURVE 以相位/圆弧边界为事件切步，积分同一个自行车模型。
if nargin<10, allow_stationary_wait=false; end
cfg = planner.vhybrid; dyn = planner.dynamics;
static_context = map.collision_context;
nodes = repmat(vhybrid_node(),0,1); reason = 'constraint';
state = [parent.x,parent.y,parent.theta,parent.v,parent.t];
gear = parent.gear;
previous = parent; segment = 1; phase = 1;
segment_remaining = curve.lengths(1); time_remaining = phases(1,1);
while segment <= 3 && phase <= size(phases,1)
    if segment_remaining <= 1e-10 && segment < 3
        segment = segment+1;
        if segment <= 3, segment_remaining = curve.lengths(segment); end
        continue;
    end
    % 相位减法可留下机器精度的正残余；此时再积分会使t+dt==t。
    % 仅跳过不可分辨的相位尾部，末速度仍由真实积分与末状态容差验收。
    phase_time_tolerance = 64*eps(max(1,abs(state(5))+time_remaining));
    if time_remaining <= phase_time_tolerance
        phase = phase+1;
        if phase <= size(phases,1), time_remaining = phases(phase,1); end
        continue;
    end
    % 根据剩余相位时间修正舍入误差；不修改状态，不超过加速度边界。
    target_speed=phases(phase,3);
    phase_acceleration=phases(phase,2);
    if allow_stationary_wait && state(4)<=1e-10 && target_speed<=1e-10 && ...
            abs(phase_acceleration)<=1e-12
        dt=min(cfg.goal_connection_step_s,time_remaining);
        next=state; next(5)=state(5)+dt;
        if day6_static_collision_core(next(1:3),static_context)
            reason='collision'; return;
        end
        [g,h]=vhybrid_cost(previous,next,goal,planner,0,0,gear,false,false);
        if isfinite(cost_limit) && isfinite(g) && g>=cost_limit
            reason='cost_bound'; return;
        end
        indices=[floor((next(1)-map.bounds(1))/cfg.xy_grid_resolution_m), ...
            floor((next(2)-map.bounds(2))/cfg.xy_grid_resolution_m), ...
            round((mod(next(3)+pi,2*pi))/cfg.yaw_grid_resolution_rad), ...
            round(next(4)/cfg.velocity_grid_resolution_mps), ...
            round(next(5)/cfg.time_grid_resolution_s)];
        node=vhybrid_node(next,0,0,previous.node_id,g,h,indices,0,gear,false);
        node.direction=0; node.travelled_distance=0;
        nodes(end+1,1)=node; %#ok<AGROW>
        previous=node; state=next; time_remaining=time_remaining-dt;
        continue;
    end
    acceleration = (target_speed-state(4))/time_remaining;
    acceleration = max(dyn.a_min_mps2,min(dyn.a_max_mps2,acceleration));
    steering = gear*curve.types(segment)*actual_delta;
    dt = min(cfg.goal_connection_step_s,time_remaining);
    ds_limit = min(segment_remaining,cfg.collision_check_step_m);
    if segment == 3 && segment_remaining <= 1e-10
        % 最末段不能因弧长舍入提前结束制动，仍积分完整相位直到 v=0。
        ds_limit = cfg.collision_check_step_m;
    end
    possible_distance = state(4)*dt+0.5*acceleration*dt^2;
    if possible_distance > ds_limit
        % 稳定解法，避免减速时求根的数值抵消。
        end_speed = sqrt(max(0,state(4)^2+2*acceleration*ds_limit));
        dt = min(dt,2*ds_limit/(state(4)+end_speed));
    end
    [next,ds,~,valid,lateral_detail] = summon_vehicle_dynamic_gear( ...
        state,steering,acceleration,dt,vehicle,planner,gear);
    if ~valid
        if isfield(lateral_detail,'enabled') && lateral_detail.enabled && ...
                isLateralLimitReason(lateral_detail.reason)
            reason = lateral_detail.reason;
        end
        return;
    end
    if abs(ds) <= 0 || next(5) <= state(5)
        return;
    end
    if day6_static_collision_core(next(1:3),static_context)
        reason = 'collision'; return;
    end
    [g,h] = vhybrid_cost(previous,next,goal,planner,ds,steering,gear,false,false);
    % 已付代价也是完整路径的下界；非负项的后续累加不能再降低g。
    % 不使用h/f，也不假设每个航向增量小于pi。
    if isfinite(cost_limit) && isfinite(g) && g >= cost_limit
        reason = 'cost_bound'; return;
    end
    indices = [floor((next(1)-map.bounds(1))/cfg.xy_grid_resolution_m), ...
        floor((next(2)-map.bounds(2))/cfg.xy_grid_resolution_m), ...
        round((mod(next(3)+pi,2*pi))/cfg.yaw_grid_resolution_rad), ...
        round(next(4)/cfg.velocity_grid_resolution_mps),round(next(5)/cfg.time_grid_resolution_s)];
    node = vhybrid_node(next,acceleration,steering,previous.node_id,g,h,indices,0,gear,false);
    node.direction = gear; node.travelled_distance = ds;
    nodes(end+1,1) = node; %#ok<AGROW>
    previous = node; state = next;
    segment_remaining = segment_remaining-abs(ds); time_remaining = time_remaining-dt;
end
% 验收模型积分的真实末状态，绝不将节点替换成目标。
if ~isempty(nodes) && hypot(state(1)-goal(1),state(2)-goal(2)) <= cfg.goal_position_tolerance_m && ...
        abs(mod(state(3)-goal(3)+pi,2*pi)-pi) <= cfg.goal_heading_tolerance_rad && ...
        abs(state(4)-cfg.terminal_speed_mps) <= cfg.goal_speed_tolerance_mps
    reason = 'ok';
end
end

function config = readMultiPhaseConfig(cfg,dyn)
%READMULTIPHASECONFIG 读取可选的停车等待重加速候选参数。
required={'goal_connection_multi_phase_cruise_speeds_mps', ...
    'goal_connection_multi_phase_stop_distance_fractions'};
for k=1:numel(required)
    if ~isfield(cfg,required{k}) || ~numericVector(cfg.(required{k}))
        error('VHybrid:InvalidMultiPhaseProfile','多相位速度和停车比例必须为非空有限实数向量。');
    end
end
config.cruise_speeds_mps = cfg.goal_connection_multi_phase_cruise_speeds_mps(:).';
config.stop_distance_fractions = cfg.goal_connection_multi_phase_stop_distance_fractions(:).';
if isfield(cfg,'goal_connection_multi_phase_wait_times_s') && ...
        ~isempty(cfg.goal_connection_multi_phase_wait_times_s)
    if ~numericVector(cfg.goal_connection_multi_phase_wait_times_s)
        error('VHybrid:InvalidMultiPhaseProfile','等待时间必须为有限非负实数向量。');
    end
    config.wait_times_s = cfg.goal_connection_multi_phase_wait_times_s(:).';
else
    % 等待时间可选：缺省仅生成“减速停车后立即重加速”的剖面。
    config.wait_times_s = 0;
end
values = [config.cruise_speeds_mps,config.stop_distance_fractions,config.wait_times_s];
if isempty(config.cruise_speeds_mps) || isempty(config.stop_distance_fractions) || ...
        any(~isfinite(values)) || ...
        any(config.cruise_speeds_mps<=0) || any(config.cruise_speeds_mps>dyn.v_max_mps) || ...
        any(config.stop_distance_fractions<=0) || any(config.stop_distance_fractions>=1) || ...
        any(config.wait_times_s<0) || dyn.v_min_mps>0
    error('VHybrid:InvalidMultiPhaseProfile','Day7多相位速度剖面参数无效。');
end
end

function valid = numericVector(value)
valid=isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value(:)));
end

function [candidates,profile_attempts] = vhybrid_multi_phase_profiles(distance,v0,vf, ...
    cruise_speeds,stop_fractions,wait_times,dyn,segment_lengths, ...
    segment_curvatures,lateral_enabled,max_lateral_acceleration,tolerance)
%VHYBRIDMULTIPHASEPROFILES 生成固定几何上的停车等待重加速剖面。
candidates = struct('phases',{},'wait_time_s',{}, ...
    'cruise_speed_mps',{},'stop_distance_fraction',{},'profile_diagnostics',{});
profile_attempts = {};
ap=dyn.a_max_mps2; ab=-dyn.a_min_mps2; vmin=dyn.v_min_mps; vmax=dyn.v_max_mps;
if isempty(wait_times), wait_times=0; end
if nargin<8, segment_lengths=zeros(1,0); end
if nargin<9, segment_curvatures=zeros(1,0); end
if nargin<10, lateral_enabled=false; end
if nargin<11, max_lateral_acceleration=NaN; end
if nargin<12, tolerance=0; end
for cruise_speed=cruise_speeds
    for fraction=stop_fractions
        stop_distance=fraction*distance;
        for wait_time=wait_times
            if lateral_enabled
                [phases,feasible,diagnostic]=vhybrid_segmented_speed_phases( ...
                    segment_lengths,segment_curvatures,v0,vf,cruise_speed,dyn, ...
                    max_lateral_acceleration,tolerance,stop_distance,wait_time);
                diagnostic.candidate_kind='multi_phase';
                profile_attempts{end+1,1}=diagnostic; %#ok<AGROW>
            else
                phases=buildMultiPhaseSegments(distance,v0,vf,cruise_speed, ...
                    stop_distance,wait_time,ap,ab,vmin,vmax);
                feasible=~isempty(phases);
                diagnostic=struct();
            end
            if feasible
                candidates(end+1,1)=struct('phases',phases, ...
                    'wait_time_s',wait_time,'cruise_speed_mps',cruise_speed, ...
                    'stop_distance_fraction',fraction, ...
                    'profile_diagnostics',diagnostic); %#ok<AGROW>
            end
        end
    end
end
end

function phases = buildMultiPhaseSegments(distance,v0,vf,cruise_speed, ...
    stop_distance,wait_time,ap,ab,vmin,vmax)
%BUILDMULTIPHASESEGMENTS 解析构造速度连续、弧长精确的多相位剖面。
phases=zeros(0,3);
if v0<vmin-1e-9 || vf<vmin-1e-9 || cruise_speed<vmin || ...
        max([v0,vf,cruise_speed])>vmax+1e-9 || cruise_speed<v0-1e-9 || ...
        vf>cruise_speed+1e-9 || ap<=0 || ab<=0
    return;
end
ramp_up=(cruise_speed^2-v0^2)/(2*ap);
brake_to_stop=cruise_speed^2/(2*ab);
ramp_after_stop=cruise_speed^2/(2*ap);
terminal_brake=(cruise_speed^2-vf^2)/(2*ab);
if ramp_up+brake_to_stop>stop_distance+1e-9 || ...
        ramp_after_stop+terminal_brake>distance-stop_distance+1e-9
    return;
end
cruise_before=max(0,stop_distance-ramp_up-brake_to_stop);
cruise_after=max(0,distance-stop_distance-ramp_after_stop-terminal_brake);
phases=[(cruise_speed-v0)/ap,ap,cruise_speed; ...
    cruise_before/cruise_speed,0,cruise_speed; ...
    cruise_speed/ab,-ab,0];
if wait_time>0
    phases=[phases;wait_time,0,0]; %#ok<AGROW>
end
phases=[phases;cruise_speed/ap,ap,cruise_speed; ...
    cruise_after/cruise_speed,0,cruise_speed; ...
    (cruise_speed-vf)/ab,-ab,vf]; %#ok<AGROW>
speed=v0; travelled=0;
for k=1:size(phases,1)
    duration=phases(k,1); acceleration=phases(k,2); target=phases(k,3);
    if duration < -1e-10 || ~isfinite(duration)
        phases=zeros(0,3); return;
    end
    next_speed=speed+acceleration*duration;
    travelled=travelled+0.5*(speed+next_speed)*duration;
    if abs(next_speed-target)>1e-8*max(1,target)
        phases=zeros(0,3); return;
    end
    speed=next_speed;
end
if abs(speed-vf)>1e-8 || abs(travelled-distance)>1e-6*max(1,distance)
    phases=zeros(0,3);
end
% 零时长相位不应生成重复时间节点。
phases=phases(phases(:,1)>0,:);
end

function enabled = nonnegativeConnectionCosts(cfg,gear,parent_cost)
%NONNEGATIVECONNECTIONCOSTS 检查vhybrid_cost中同挡连接实际使用的系数。
names = {'weight_distance','weight_speed_change','weight_reference_speed', ...
    'weight_heading_change','weight_time'};
enabled = finiteRealScalar(parent_cost) && finiteRealScalar(cfg.reference_speed_mps);
for k = 1:numel(names)
    value = cfg.(names{k});
    if ~finiteRealScalar(value) || value < 0
        enabled = false; return;
    end
end
if gear < 0 && isfield(cfg,'reverse_penalty')
    enabled = enabled && finiteRealScalar(cfg.reverse_penalty) && cfg.reverse_penalty >= 0;
end
end

function valid = finiteRealScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
