function [children, statistics] = vhybrid_expand_node(parent_node, map, ...
    vehicle_config, planner_config, goal_state, st_map, higher_priority_trajectories)
%VHYBRID_EXPAND_NODE 对节点采样加速度和转角并生成子节点。
%
% 输入：
%   parent_node    - vhybrid_node 结构体，状态为 [x,y,theta,v,t]。
%   map            - summon_map 输出的静态地图。
%   vehicle_config - 车辆尺寸和转角配置。
%   planner_config - dynamics/vhybrid 配置。
%   goal_state     - 真实目标状态 [x,y,theta,v,t]，用于启发代价。
%
% 输出：
%   st_map/higher_priority_trajectories - 可选时空资源表与高优先级车辆。
%   children       - 通过运动学、静态和连续时空碰撞检查的子节点数组。
%   statistics     - 采样、裁剪、减速中间节点和按原因分类的统计。
%
% 算法逻辑：
%   按配置组合加速度和转角；v 表示速度大小，gear 表示运动方向。
%   使用 summon_vehicle_dynamic_gear 适配 Day3 模型进行有符号圆弧积分。
%   仅在停稳后生成固定姿态的换挡驻留边，查重额外保留挡位。

cfg = planner_config.vhybrid;
dynamics = planner_config.dynamics;
% 本次扩展内地图/尺寸保持不变；入口核对一次，采样内只运行几何内核。
map = prepare_static_context(map,vehicle_config);
static_context = map.collision_context;
if nargin < 6, st_map = []; end
if nargin < 7, higher_priority_trajectories = []; end
has_context = ~isempty(st_map) || ~isempty(higher_priority_trajectories);
gear = 1;
if isfield(parent_node,'gear'), gear = parent_node.gear; end
reverse_enabled = isfield(cfg,'reverse_enabled') && cfg.reverse_enabled;
if isfield(cfg, 'control_acceleration_samples_mps2')
    acceleration_values = double(cfg.control_acceleration_samples_mps2(:).');
else
    acceleration_values = [-dynamics.a_max_mps2, 0, dynamics.a_max_mps2];
end
if isfield(cfg, 'control_steering_samples_rad')
    steering_values = double(cfg.control_steering_samples_rad(:).');
else
    steering_values = [-vehicle_config.steering.maximum_steer_rad, 0, ...
        vehicle_config.steering.maximum_steer_rad];
end
children = repmat(vhybrid_node(), 0, 1);
statistics = struct('sampled', 0, 'valid', 0, 'collision_pruned', 0, ...
    'constraint_pruned', 0, 'dynamic_pruned',0,'resource_pruned',0, ...
    'braking_midpoint_pruned',0,'faster_controls_skipped',0,'waiting_nodes',0,'horizon_pruned',0, ...
    'reason_counts',struct(),'conflict_examples',{{}}, ...
    'reverse_nodes',0,'gear_switch_nodes',0,'gear_switch_pruned',0);
% 同一转向按下一时刻速度由低到高扩展。减速中间节点不安全时，
% 该转向更快的控制不再扩展；不同转向仍独立搜索。
acceleration_values = sort(acceleration_values);

for steering_angle = steering_values
    for acceleration_id = 1:numel(acceleration_values)
        acceleration = acceleration_values(acceleration_id);
        statistics.sampled = statistics.sampled + 1;
        current_state = [parent_node.x, parent_node.y, parent_node.theta, ...
            parent_node.v, parent_node.t];
        if has_context && isfield(planner_config,'day6') && ...
                planner_config.day6.require_immediate_start && ...
                parent_node.parent_id == 0 && parent_node.v == 0 && acceleration <= 0
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            statistics = countReason(statistics,'initial_motion_required');
            continue;
        end
        [next_state, distance, ~, valid] = summon_vehicle_dynamic_gear( ...
            current_state,steering_angle,acceleration,cfg.time_step_s, ...
            vehicle_config,planner_config,gear);
        if ~valid
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            statistics = countReason(statistics,'kinematic_constraint');
            continue;
        end
        if has_context && ~isempty(st_map) && next_state(5) >= st_map.t_max
            statistics.horizon_pruned = statistics.horizon_pruned+1;
            statistics.constraint_pruned = statistics.constraint_pruned+1;
            statistics = countReason(statistics,'time_horizon_exhausted');
            continue;
        end
        % 中间状态沿真实圆弧积分，禁止用直线 xy/yaw 插值代替扫掠轨迹。
        sample_count = max(2,ceil(abs(distance)/cfg.collision_check_step_m));
        if has_context
            sample_count = max(sample_count,ceil(cfg.time_step_s / ...
                planner_config.day6.collision_sample_step_s));
            % 减速段额外插入检测节点，即使位移很短也不会只检查末端。
            if acceleration < 0, sample_count = max(4,2*sample_count); end
        end
        collision = false;
        intermediate_collision = false;
        if has_context
            % 保存同一控制已经积分并接受的静态样本，动态检测可按同时间复用。
            static_states = zeros(sample_count+1,5);
            static_states(1,:) = current_state;
        end
        for sample_id = 1:sample_count
            ratio = sample_id/sample_count;
            [sample_state,~,~,sample_valid] = summon_vehicle_dynamic_gear( ...
                current_state,steering_angle,acceleration,cfg.time_step_s*ratio, ...
                vehicle_config,planner_config,gear);
            if ~sample_valid
                collision = true; break;
            end
            if has_context, static_states(sample_id+1,:) = sample_state; end
            if day6_static_collision_core(sample_state(1:3),static_context)
                collision = true;
                intermediate_collision = sample_id < sample_count;
                break;
            end
        end
        if collision
            statistics.collision_pruned = statistics.collision_pruned + 1;
            statistics = countReason(statistics,'static_collision');
            if has_context && acceleration < 0 && intermediate_collision
                statistics.braking_midpoint_pruned = statistics.braking_midpoint_pruned + 1;
                statistics.faster_controls_skipped = statistics.faster_controls_skipped + ...
                    numel(acceleration_values)-acceleration_id;
                break;
            end
            continue;
        end

        if has_context
            motion = day6_motion_trajectory(current_state,next_state,steering_angle, ...
                acceleration,vehicle_config,planner_config,gear,static_states);
            [collision,report] = check_trajectory_conflict(motion, ...
                higher_priority_trajectories,vehicle_config,planner_config,st_map, ...
                struct('collect_distance_samples',false));
            if collision
                statistics.collision_pruned = statistics.collision_pruned + 1;
                % 保守动态资源冲突同样是动态拒绝；纯时空范围越界另行统计。
                statistics.dynamic_pruned = statistics.dynamic_pruned + double( ...
                    report.rectangle_collision || report.physical_collision || ...
                    (report.resource_conflict && ~strcmp(report.reason,'outside_time_space_bounds')));
                statistics.resource_pruned = statistics.resource_pruned + double(report.resource_conflict);
                statistics = countReason(statistics,report.reason);
                if numel(statistics.conflict_examples) < planner_config.day6.max_conflict_examples
                    statistics.conflict_examples{end+1} = report;
                end
                if acceleration < 0 && isfield(report,'has_intermediate_conflict') && ...
                        report.has_intermediate_conflict
                    statistics.braking_midpoint_pruned = statistics.braking_midpoint_pruned + 1;
                    statistics.faster_controls_skipped = statistics.faster_controls_skipped + ...
                        numel(acceleration_values)-acceleration_id;
                    break;
                end
                continue;
            end
        end

        indices = discretizeState(next_state, map, cfg);
        [g_cost, h_cost, ~, ~] = vhybrid_cost(parent_node, next_state, ...
            goal_state, ...
            planner_config, distance, steering_angle,gear,false);
        child = vhybrid_node(next_state, acceleration, steering_angle, ...
            parent_node.node_id, g_cost, h_cost, indices, 0,gear,false);
        child.direction = sign(distance);
        child.travelled_distance = distance;
        children(end+1,1) = child; %#ok<AGROW>
        statistics.valid = statistics.valid + 1;
        statistics.reverse_nodes = statistics.reverse_nodes + double(gear<0 && abs(distance)>1e-12);
        if abs(distance) < 1e-12 && next_state(5) > current_state(5)
            statistics.waiting_nodes = statistics.waiting_nodes + 1;
        end
    end
end

% 换挡必须由静止节点出发。它是完整的占用时间段，不是直接修改运动方向。
% Day6 要求两车立即从 t=0 起步，因此根节点不能以换挡驻留代替起步。
initial_must_move = has_context && isfield(planner_config,'day6') && ...
    planner_config.day6.require_immediate_start && parent_node.parent_id==0;
if reverse_enabled && parent_node.v==0 && ~initial_must_move
    if ~isfield(dynamics,'direction_change_time_s') || ...
            ~isscalar(dynamics.direction_change_time_s) || ...
            ~isfinite(dynamics.direction_change_time_s) || dynamics.direction_change_time_s<=0
        error('VHybrid:InvalidGearChangeTime','换挡等待时间必须集中配置为正有限秒数。');
    end
    statistics.sampled = statistics.sampled+1;
    current = [parent_node.x,parent_node.y,parent_node.theta,0,parent_node.t];
    next = current; next(5) = current(5)+dynamics.direction_change_time_s;
    blocked = day6_static_collision_core(current(1:3),static_context);
    reason = 'static_collision';
    if ~isempty(st_map) && next(5)>=st_map.t_max
        blocked = true; reason = 'time_horizon_exhausted';
        statistics.horizon_pruned = statistics.horizon_pruned+1;
    end
    if ~blocked && has_context
        dwell = struct('x',[current(1);next(1)],'y',[current(2);next(2)], ...
            'theta',[current(3);next(3)],'v',[0;0],'t',[current(5);next(5)], ...
            'acceleration',[0;0],'steering_angle',[0;0], ...
            'gear',[gear;-gear],'is_gear_change',[false;true]);
        [blocked,report] = check_trajectory_conflict(dwell,higher_priority_trajectories, ...
            vehicle_config,planner_config,st_map,struct('collect_distance_samples',false));
        if blocked
            reason = report.reason;
            statistics.dynamic_pruned = statistics.dynamic_pruned+double( ...
                report.rectangle_collision || report.physical_collision || report.resource_conflict);
            statistics.resource_pruned = statistics.resource_pruned+double(report.resource_conflict);
        end
    end
    if blocked
        statistics.gear_switch_pruned = statistics.gear_switch_pruned+1;
        statistics.collision_pruned = statistics.collision_pruned+1;
        statistics = countReason(statistics,reason);
    else
        indices = discretizeState(next,map,cfg);
        [g,h] = vhybrid_cost(parent_node,next,goal_state,planner_config,0,0,-gear,true);
        child = vhybrid_node(next,0,0,parent_node.node_id,g,h,indices,0,-gear,true);
        children(end+1,1) = child;
        statistics.valid = statistics.valid+1;
        statistics.gear_switch_nodes = statistics.gear_switch_nodes+1;
        statistics.waiting_nodes = statistics.waiting_nodes+1;
    end
end

function statistics = countReason(statistics,reason)
%COUNTREASON 将冲突原因转换为合法字段，并统计每种拒绝的节点数。
key = matlab.lang.makeValidName(char(reason));
if ~isfield(statistics.reason_counts,key), statistics.reason_counts.(key) = 0; end
statistics.reason_counts.(key) = statistics.reason_counts.(key)+1;
end
end

function indices = discretizeState(state, map, cfg)
%DISCRETIZESTATE 将连续状态离散为五维查重索引。
indices = [floor((state(1) - map.bounds(1)) / cfg.xy_grid_resolution_m), ...
    floor((state(2) - map.bounds(2)) / cfg.xy_grid_resolution_m), ...
    round((wrapToPiLocal(state(3)) + pi) / cfg.yaw_grid_resolution_rad), ...
    round(state(4) / cfg.velocity_grid_resolution_mps), ...
    round(state(5) / cfg.time_grid_resolution_s)];
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将航向角归一化到 [-pi,pi)。
angle = mod(angle + pi, 2*pi) - pi;
end
