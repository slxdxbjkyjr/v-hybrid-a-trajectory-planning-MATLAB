function result = run_day7_plan_c_experiment(experiment_config_path)
%RUN_DAY7_PLAN_C_EXPERIMENT 固定既有四车路线的方案C真实控制剖面对照。
% 输入：独立实验JSON，缺省读取config/day7_plan_c_experiment.json。
% 输出：四车轨迹、逐车/全车对验收、剖面耗时、对照表和Day7同款动画。
% 原搜索轨迹作为真实基线；仅替换002从指定父节点到终点的速度剖面。
% 不延迟t=0起步，不改变安全判据、车辆约束或其他三车的路线/时间。
project = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1
    experiment_config_path=fullfile(project,'config','day7_plan_c_experiment.json');
end
experiment=jsondecode(fileread(experiment_config_path));
planner=jsondecode(fileread(fullfile(project,'config','planner_config_day7.json')));
vehicle=jsondecode(fileread(fullfile(project,'config','vehicle_config_day7.json')));
map_config=jsondecode(fileread(fullfile(project,'config','map_config_day7.json')));
geometry=summon_map(map_config);
baseline_file=fullfile(project,experiment.baseline_trajectory_file);
baseline=load(baseline_file,'paths','result','timings');
assert(baseline.result.valid && numel(baseline.paths)==4, ...
    'Day7PlanC:InvalidBaseline','需要既有四车完整验收通过的真实搜索结果。');
records=baseline.result.per_vehicle;
ids={records.id}; ego=find(strcmp(ids,experiment.vehicle_id),1);
assert(~isempty(ego),'Day7PlanC:MissingVehicle','基线缺少指定车辆。');
original=baseline.paths{ego};
planner.vhybrid.heuristic_weight=planner.day6.search_heuristic_weight;
planner.vhybrid.heuristic_method=planner.day6.heuristic_method;
delta=min(vehicle.steering.maximum_steer_rad,planner.dynamics.delta_max_rad);
planner.vhybrid.minimum_turning_radius_m=vehicle.dimensions_m.wheelbase/tan(delta);
names=fieldnames(experiment.vhybrid);
for k=1:numel(names)
    planner.vhybrid.(names{k})=experiment.vhybrid.(names{k});
end
planner.st_occupancy.trajectory_end_policy='hold_until_t_max';
planner.st_occupancy.safety_distance_m=max(planner.st_occupancy.safety_distance_m, ...
    planner.day6.minimum_safety_distance_m/2);
output_dir=fullfile(project,experiment.output_directory);
assert(~strcmp(output_dir,fileparts(baseline_file)), ...
    'Day7PlanC:OutputOverwrite','实验目录必须与基线输出目录分离。');
if ~exist(output_dir,'dir'),mkdir(output_dir);end
% 保存真实基线及配置快照，后续重跑基线也不会丢失本轮对照依据。
save(fullfile(output_dir,'day7_plan_c_baseline.mat'),'baseline','planner', ...
    'vehicle','map_config','experiment','-v7');

st_map=st_occupancy_map(planner,geometry);
st_map=mark_static_obstacles(st_map,geometry,vehicle);
higher=struct('id',{},'states',{},'vehicle_config',{},'footprint',{},'hold_end',{});
for k=1:numel(records)
    if records(k).effective_priority>=records(ego).effective_priority,continue;end
    st_map=mark_trajectory_occupancy(st_map,baseline.paths{k},vehicle,ids{k});
    higher(end+1,1)=dynamic_vehicle_obstacle(baseline.paths{k},vehicle,ids{k},planner); %#ok<AGROW>
end
st_map=prepare_dynamic_context(st_map,higher);
i0=find(abs(original.t-experiment.connection_start_time_s)<1e-8,1);
assert(~isempty(i0),'Day7PlanC:InvalidParentTime','指定时间必须对应原轨迹真实节点。');
parent=vhybrid_node([original.x(i0),original.y(i0),original.theta(i0), ...
    original.v(i0),original.t(i0)],0,0,0,0,0,zeros(1,5),1,original.gear(i0));
goal=[original.x(end),original.y(end),original.theta(end),original.v(end),0];
timer=tic;
[ok,nodes,connection_stats]=vhybrid_goal_connection(parent,goal, ...
    geometry,vehicle,planner,st_map,higher);
profile_seconds=toc(timer);
result=struct('valid',false,'reason','connection_failed', ...
    'experiment',experiment.experiment,'checked_at',char(datetime('now')), ...
    'baseline_file',baseline_file,'planning_order',{ids},'configuration',experiment, ...
    'vehicle_config',vehicle,'planner_config',planner,'map_config',map_config, ...
    'profile_planning_time_s',profile_seconds,'connection_statistics',connection_stats, ...
    'paths',{baseline.paths},'per_vehicle',records,'pairwise_validation',struct([]), ...
    'route_validation',struct(),'output_directory',output_dir);
if ~ok
    writeReport(result,output_dir);
    fprintf('Plan C failed: %s. See %s\n',result.reason,output_dir);
    return;
end
updated=original;
sample_fields={'x','y','theta','v','t','acceleration','steering_angle', ...
    'gear','is_gear_change','travelled_distance','direction'};
for k=1:numel(sample_fields)
    f=sample_fields{k};
    updated.(f)=[original.(f)(1:i0);vertcat(nodes.(f))];
end
updated.search_nodes=repmat(vhybrid_node(),0,1);
updated.kinematic_validation=vhybrid_validate_trajectory(updated,geometry,vehicle,planner);
updated.final_position_error_m=hypot(updated.x(end)-goal(1),updated.y(end)-goal(2));
updated.final_heading_error_rad=abs(wrapAngle(updated.theta(end)-goal(3)));
updated.final_speed_error_mps=abs(updated.v(end)-goal(4));
updated.valid=updated.kinematic_validation.valid && ...
    updated.final_position_error_m<=planner.vhybrid.goal_position_tolerance_m && ...
    updated.final_heading_error_rad<=planner.vhybrid.goal_heading_tolerance_rad && ...
    updated.final_speed_error_mps<=planner.vhybrid.goal_speed_tolerance_mps;
updated.search_statistics.plan_c_profile_time_s=profile_seconds;
updated.search_statistics.plan_c_connection=connection_stats;
updated.plan_c_fixed_route_experiment=true;
% 原搜索的代价/耗时仍保留在baseline快照，不把剖面时间冒充整车搜索耗时。
updated.search_statistics.baseline_final_solution_cost= ...
    updated.search_statistics.final_solution_cost;
updated.search_statistics.final_solution_cost=trajectoryCost(updated,goal,planner);
result.paths{ego}=updated;
result.route_validation=compareRoute(original,updated,vehicle,planner,experiment);

all_vehicle_ok=true;
for k=1:numel(records)
    path=result.paths{k};
    replay=vhybrid_validate_trajectory(path,geometry,vehicle,planner);
    immediate=path.t(1)==0 && path.v(1)==0 && ...
        abs(path.travelled_distance(2))>0 && path.v(2)>0;
    terminal=abs(path.v(end))<=planner.vhybrid.goal_speed_tolerance_mps;
    result.per_vehicle(k).kinematic_valid=replay.valid;
    result.per_vehicle(k).starts_at_rest_t0=path.t(1)==0 && path.v(1)==0;
    result.per_vehicle(k).moves_immediately=immediate;
    result.per_vehicle(k).terminal_speed_zero=terminal;
    result.per_vehicle(k).valid=path.valid && replay.valid && immediate && terminal;
    result.per_vehicle(k).final_state=[path.x(end),path.y(end),path.theta(end),path.v(end),path.t(end)];
    all_vehicle_ok=all_vehicle_ok && result.per_vehicle(k).valid;
end
result.per_vehicle(ego).search_statistics=updated.search_statistics;
result.per_vehicle(ego).planning_time_s=records(ego).planning_time_s+profile_seconds;
result.per_vehicle(ego).reason='passed_plan_c_fixed_route';
[pairs,pairs_ok]=validatePairs(result.paths,ids,geometry,vehicle,planner);
result.pairwise_validation=pairs;
result.valid=all_vehicle_ok && pairs_ok && result.route_validation.valid;
if result.valid,result.reason='passed';else,result.reason='validation_failed';end

old_metrics=speedMetrics(original,experiment.low_speed_threshold_mps);
new_metrics=speedMetrics(updated,experiment.low_speed_threshold_mps);
result.comparison=struct('baseline',old_metrics,'plan_c',new_metrics, ...
    'arrival_reduction_s',old_metrics.arrival_time_s-new_metrics.arrival_time_s, ...
    'arrival_reduction_percent',100*(1-new_metrics.arrival_time_s/old_metrics.arrival_time_s), ...
    'other_vehicle_paths_unchanged',isequaln(result.paths([1:ego-1,ego+1:4]), ...
    baseline.paths([1:ego-1,ego+1:4])));
result.timings=struct('baseline_search_total_s',baseline.timings.planning_s, ...
    'additional_profile_s',profile_seconds, ...
    'total_search_plus_profile_s',baseline.timings.planning_s+profile_seconds);
comparison_table=struct2table([old_metrics;new_metrics]);
comparison_table.variant=["baseline";"plan_c"];
writetable(comparison_table,fullfile(output_dir,'day7_plan_c_comparison.csv'),'Encoding','UTF-8');
result.output_files=struct();
if result.valid
    [result.output_files,result.output_timings]=day7_write_animation_and_tables( ...
        result.paths,result.per_vehicle,geometry,vehicle,planner,output_dir,true);
    makeComparisonFigure(original,updated,output_dir);
end
report=rmfield(result,'paths');paths=result.paths; %#ok<NASGU>
save(fullfile(output_dir,'day7_plan_c_trajectories.mat'),'paths','report','-v7');
writeReport(result,output_dir);
fprintf('Plan C %s: arrival %.6f -> %.6f s (%.2f%%), profile %.3f s.\n', ...
    result.reason,old_metrics.arrival_time_s,new_metrics.arrival_time_s, ...
    result.comparison.arrival_reduction_percent,profile_seconds);
fprintf('Fixed route residual: %.3g m / %.3g rad. Pair checks: %d/6.\n', ...
    result.route_validation.max_position_residual_m, ...
    result.route_validation.max_heading_residual_rad,nnz([pairs.valid]));
end

function route=compareRoute(a,b,vehicle,planner,cfg)
% 按相同累计弧长用实际自行车控制求位姿，避免折线插值误差和静止重复点。
length_a=sum(abs(a.travelled_distance));length_b=sum(abs(b.travelled_distance));
s=unique([(0:cfg.route_sample_step_m:min(length_a,length_b)).';min(length_a,length_b)]);
a_pose=poseAtDistance(a,s,vehicle,planner);b_pose=poseAtDistance(b,s,vehicle,planner);
route=struct('length_original_m',length_a,'length_updated_m',length_b, ...
    'max_position_residual_m',max(hypot(a_pose(:,1)-b_pose(:,1),a_pose(:,2)-b_pose(:,2))), ...
    'max_heading_residual_rad',max(abs(wrapAngle(a_pose(:,3)-b_pose(:,3)))), ...
    'valid',false);
route.valid=abs(length_a-length_b)<=cfg.route_position_tolerance_m && ...
    route.max_position_residual_m<=cfg.route_position_tolerance_m && ...
    route.max_heading_residual_rad<=cfg.route_heading_tolerance_rad;
end

function poses=poseAtDistance(path,distances,vehicle,planner)
arc=cumsum(abs(path.travelled_distance(:)));poses=zeros(numel(distances),3);
for k=1:numel(distances)
    index=find(arc<=distances(k)+1e-12,1,'last');
    state=[path.x(index),path.y(index),path.theta(index),path.v(index),path.t(index)];
    if index<numel(arc) && distances(k)>arc(index)+1e-12
        ds=distances(k)-arc(index);a=path.acceleration(index+1);
        v1=sqrt(max(0,state(4)^2+2*a*ds));dt=2*ds/(state(4)+v1);
        [state,~,~,valid]=summon_vehicle_dynamic_gear(state, ...
            path.steering_angle(index+1),a,dt,vehicle,planner,path.gear(index+1));
        assert(valid,'Day7PlanC:RouteReplay','弧长控制重放失败。');
    end
    poses(k,:)=state(1:3);
end
end

function [pairs,all_ok]=validatePairs(paths,ids,geometry,vehicle,planner)
horizon=planner.st_occupancy.t_max-1e-9;
pairs=struct('vehicle_a',{},'vehicle_b',{},'rectangle_collision',{}, ...
    'resource_overlap_blocks',{},'minimum_distance_m',{},'reason',{}, ...
    'conflict_time_s',{},'conflict_position_xy',{},'valid',{});
all_ok=true;
for i=1:3
    a=paths{i};
    % 保留运动控制逐边连续重放；额外一段零控制检查终点驻车至地图末层。
    for j=i+1:4
        obstacle=dynamic_vehicle_obstacle(paths{j},vehicle,ids{j},planner);
        [hit,detail]=check_trajectory_conflict(a,obstacle,vehicle,planner,[]);
        hold_path=heldGoal(a,horizon);
        [hold_hit,hold_detail]=check_trajectory_conflict(hold_path,obstacle,vehicle,planner,[]);
        if hold_hit && ~hit,detail=hold_detail;end
        map=st_occupancy_map(planner,geometry);
        map=mark_trajectory_occupancy(map,a,vehicle,ids{i});
        [~,stats]=mark_trajectory_occupancy(map,paths{j},vehicle,ids{j});
        valid=~hit && ~hold_hit && stats.conflict_blocks==0;
        pairs(end+1,1)=struct('vehicle_a',ids{i},'vehicle_b',ids{j}, ...
            'rectangle_collision',hit || hold_hit,'resource_overlap_blocks',stats.conflict_blocks, ...
            'minimum_distance_m',min(detail.minimum_distance_m,hold_detail.minimum_distance_m), ...
            'reason',detail.reason,'conflict_time_s',detail.conflict_time_s, ...
            'conflict_position_xy',detail.conflict_position_xy,'valid',valid); %#ok<AGROW>
        all_ok=all_ok && valid;
    end
end
end

function path=heldGoal(a,horizon)
path=struct('x',[a.x(end);a.x(end)],'y',[a.y(end);a.y(end)], ...
    'theta',[a.theta(end);a.theta(end)],'v',[0;0], ...
    't',[a.t(end);horizon],'acceleration',[0;0],'steering_angle',[0;0], ...
    'gear',[a.gear(end);a.gear(end)],'is_gear_change',[false;false]);
end

function metrics=speedMetrics(path,threshold)
% 逐段线性速度的精确时间积分；低速包括停车，同时单独列出静止时间。
low=0;stopped=0;longest=0;continuous=0;
for k=2:numel(path.t)
    dt=path.t(k)-path.t(k-1);v0=path.v(k-1);v1=path.v(k);
    if max(v0,v1)<=threshold
        portion=dt;continuous=continuous+portion;
    elseif min(v0,v1)>threshold
        portion=0;continuous=0;
    else
        portion=dt*(threshold-min(v0,v1))/abs(v1-v0);
        if v1<v0,continuous=portion;
        else,continuous=continuous+portion;end
    end
    low=low+portion;longest=max(longest,continuous);
    if v1>threshold,continuous=0;end
    if max(v0,v1)<=1e-8,stopped=stopped+dt;end
end
metrics=struct('arrival_time_s',path.t(end),'path_length_m',sum(abs(path.travelled_distance)), ...
    'low_speed_total_time_s',low,'longest_low_speed_time_s',longest, ...
    'stationary_time_s',stopped,'max_speed_mps',max(path.v));
end

function cost=trajectoryCost(path,goal,planner)
parent=vhybrid_node([path.x(1),path.y(1),path.theta(1),path.v(1),path.t(1)], ...
    0,0,0,0,0,zeros(1,5),1,path.gear(1));
for k=2:numel(path.t)
    state=[path.x(k),path.y(k),path.theta(k),path.v(k),path.t(k)];
    [g,h]=vhybrid_cost(parent,state,goal,planner,path.travelled_distance(k), ...
        path.steering_angle(k),path.gear(k),path.is_gear_change(k));
    parent=vhybrid_node(state,path.acceleration(k),path.steering_angle(k),0,g,h, ...
        zeros(1,5),k,path.gear(k),path.is_gear_change(k));
end
cost=parent.g_cost;
end

function writeReport(result,output_dir)
report=rmfield(result,'paths');
fid=fopen(fullfile(output_dir,'day7_plan_c_result.json'),'w','n','UTF-8');
assert(fid>=0,'Day7PlanC:OutputFailure','无法创建方案C报告。');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
end

function makeComparisonFigure(a,b,output_dir)
f=figure('Visible','off','Position',[100,100,1200,460]);
cleanup=onCleanup(@()close(f)); %#ok<NASGU>
subplot(1,2,1);plot(a.x,a.y,'b-','LineWidth',2);hold on;
plot(b.x,b.y,'r--','LineWidth',1.3);axis equal;grid on;
xlabel('x (m)');ylabel('y (m)');title('vehicle 002: same geometric route');
legend('Baseline','Plan C','Location','best');
subplot(1,2,2);plot(a.t,a.v,'b-','LineWidth',1.5);hold on;
plot(b.t,b.v,'r-','LineWidth',1.5);yline(1,'k:');grid on;
xlabel('time (s)');ylabel('speed (m/s)');title('vehicle 002: speed-time comparison');
legend('Baseline','Plan C','Location','best');
exportgraphics(f,fullfile(output_dir,'day7_plan_c_comparison.png'),'Resolution',150);
end

function angle=wrapAngle(angle)
angle=mod(angle+pi,2*pi)-pi;
end

