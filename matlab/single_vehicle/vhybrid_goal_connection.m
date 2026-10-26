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
if nargin < 6, st_map = []; end
if nargin < 7, higher_priority_trajectories = []; end
has_context = ~isempty(st_map) || ~isempty(higher_priority_trajectories);
success = false;
connection_nodes = repmat(vhybrid_node(), 0, 1);
statistics = struct('candidates', 0, 'collision_pruned', 0, ...
    'constraint_pruned', 0, 'terminal_speed', NaN, 'connection_length_m', NaN, ...
    'dynamic_pruned',0,'resource_pruned',0,'reason_counts',struct(), ...
    'conflict_examples',{{}},'geometric_candidates',0,'boundary_geometry_pruned',0, ...
    'turning_geometry_pruned',0, ...
    'cost_bound_enabled',false,'cost_bound_pruned',0, ...
    'cost_bound_integration_pruned',0);
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
        [phases, feasible] = vhybrid_speed_phases(sum(curve.lengths),start(4), ...
            cfg.terminal_speed_mps,cruise_speed,dyn);
        if ~feasible
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
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
            success = true;
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


function [nodes, reason] = integrateCurve(parent, goal, curve, phases, ...
    actual_delta, map, vehicle, planner, cost_limit)
%INTEGRATECURVE 以相位/圆弧边界为事件切步，积分同一个自行车模型。
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
    acceleration = (phases(phase,3)-state(4))/time_remaining;
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
    [next,ds,~,valid] = summon_vehicle_dynamic_gear(state,steering,acceleration,dt, ...
        vehicle,planner,gear);
    if ~valid || abs(ds) <= 0 || next(5) <= state(5)
        return;
    end
    if day6_static_collision_core(next(1:3),static_context)
        reason = 'collision'; return;
    end
    [g,h] = vhybrid_cost(previous,next,goal,planner,ds,steering,gear,false);
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
