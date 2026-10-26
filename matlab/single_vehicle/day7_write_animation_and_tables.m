function [files, timings] = day7_write_animation_and_tables(paths, vehicles, geometry, vehicle, planner, output_dir, write_complete_outputs)
%DAY7_WRITE_ANIMATION_AND_TABLES 输出Day7四车同步动画与可读CSV表格。
% 逐车状态/耗时表始终写出；仅联合验收通过时输出同步动画和完整轨迹表。
if nargin < 7, write_complete_outputs = true; end
if ~exist(output_dir,'dir'), mkdir(output_dir); end

files = struct('vehicle_timing_csv',fullfile(output_dir,'day7_vehicle_timing.csv'));
timings = struct('animation_s',0,'table_output_s',0);

% 缺失或失败轨迹用NaN表示未知到达状态，保留未规划/搜索失败的明确原因。
n = numel(vehicles);
vehicle_id = strings(n,1); priority = zeros(n,1); planning_time_s = zeros(n,1);
arrival_time_s = nan(n,1); path_duration_s = nan(n,1); path_sample_count = zeros(n,1);
path_length_m = nan(n,1); total_abs_yaw_rad = nan(n,1);
expanded_nodes = zeros(n,1); generated_nodes = zeros(n,1);
dynamic_pruned = zeros(n,1); resource_pruned = zeros(n,1);
first_solution_time_s = nan(n,1); first_solution_cost = nan(n,1);
final_solution_cost = nan(n,1); solution_improvements = nan(n,1);
refinement_stop_reason = strings(n,1);
terminal_x_m = nan(n,1); terminal_y_m = nan(n,1); terminal_yaw_rad = nan(n,1);
terminal_speed_mps = nan(n,1); valid = false(n,1);
status = strings(n,1); error_code = nan(n,1);
joint_validation_passed = repmat(logical(write_complete_outputs),n,1);
for k = 1:n
    p = struct(); v = vehicles(k);
    if iscell(paths) && k <= numel(paths) && isstruct(paths{k}), p = paths{k}; end
    vehicle_id(k) = string(v.id); priority(k) = v.effective_priority;
    planning_time_s(k) = v.planning_time_s;
    valid(k) = v.valid; error_code(k) = v.error_code;
    status(k) = string(v.reason);
    if valid(k) && strlength(status(k)) == 0, status(k) = "passed"; end
    statistics = v.search_statistics;
    if isfield(statistics,'expanded_nodes'), expanded_nodes(k) = statistics.expanded_nodes; end
    if isfield(statistics,'generated_nodes'), generated_nodes(k) = statistics.generated_nodes; end
    if isfield(statistics,'dynamic_pruned'), dynamic_pruned(k) = statistics.dynamic_pruned; end
    if isfield(statistics,'resource_pruned'), resource_pruned(k) = statistics.resource_pruned; end
    if isfield(statistics,'first_solution_time_s'), first_solution_time_s(k) = statistics.first_solution_time_s; end
    if isfield(statistics,'first_solution_cost'), first_solution_cost(k) = statistics.first_solution_cost; end
    if isfield(statistics,'final_solution_cost'), final_solution_cost(k) = statistics.final_solution_cost; end
    if isfield(statistics,'solution_improvements'), solution_improvements(k) = statistics.solution_improvements; end
    if isfield(statistics,'refinement_stop_reason'), refinement_stop_reason(k) = string(statistics.refinement_stop_reason); end
    if isfield(p,'t'), path_sample_count(k) = numel(p.t); end
    if ~valid(k) || ~isfield(p,'valid') || ~p.valid || ~isfield(p,'t') || isempty(p.t)
        continue;
    end
    arrival_time_s(k) = p.t(end); path_duration_s(k) = p.t(end)-p.t(1);
    terminal_x_m(k) = p.x(end); terminal_y_m(k) = p.y(end);
    terminal_yaw_rad(k) = p.theta(end); terminal_speed_mps(k) = p.v(end);
    if isfield(p,'travelled_distance') && ~isempty(p.travelled_distance)
        path_length_m(k) = sum(abs(p.travelled_distance(:)));
    else
        path_length_m(k) = sum(hypot(diff(p.x(:)),diff(p.y(:))));
    end
    total_abs_yaw_rad(k) = sum(abs(mod(diff(p.theta(:))+pi,2*pi)-pi));
end
summary_table = table(vehicle_id,priority,planning_time_s,arrival_time_s,path_duration_s, ...
    path_length_m,total_abs_yaw_rad,path_sample_count,expanded_nodes,generated_nodes, ...
    terminal_x_m,terminal_y_m,terminal_yaw_rad,terminal_speed_mps,valid,status,error_code, ...
    joint_validation_passed,first_solution_time_s,first_solution_cost,final_solution_cost, ...
    solution_improvements,refinement_stop_reason,dynamic_pruned,resource_pruned);
table_timer = tic;
writetable(summary_table,files.vehicle_timing_csv,'Encoding','UTF-8');
timings.table_output_s = toc(table_timer);
if ~write_complete_outputs, return; end

if ~iscell(paths) || numel(paths) ~= 4 || numel(vehicles) ~= 4 || ...
        any(~cellfun(@(p)isstruct(p) && isfield(p,'valid') && p.valid && ...
        isfield(p,'t') && numel(p.t) >= 2, paths))
    error('Day7:IncompleteOutputPaths','同步动画和最终轨迹表要求四条有效完整路径。');
end
required = {'x','y','theta','v','t','acceleration','steering_angle','gear'};
for k = 1:numel(paths)
    if ~all(isfield(paths{k},required)) || ...
            any(cellfun(@(name)numel(paths{k}.(name)) ~= numel(paths{k}.t),required)) || ...
            any(~isfinite(paths{k}.t)) || any(diff(paths{k}.t) <= 0)
        error('Day7:InvalidOutputPath','%s轨迹字段缺失、长度不一致或时间不递增。',vehicles(k).id);
    end
end
files.animation_gif = fullfile(output_dir,'day7_four_vehicle_animation.gif');
files.animation_preview_png = fullfile(output_dir,'day7_animation_preview.png');
files.trajectory_samples_csv = fullfile(output_dir,'day7_trajectory_samples.csv');

sample_vehicle_id = strings(0,1); sample_priority = zeros(0,1); sample_index = zeros(0,1);
time_s = zeros(0,1); x_m = zeros(0,1); y_m = zeros(0,1); yaw_rad = zeros(0,1);
speed_mps = zeros(0,1); signed_speed_mps = zeros(0,1); acceleration_mps2 = zeros(0,1);
steering_angle_rad = zeros(0,1); gear = zeros(0,1);
for k = 1:n
    p = paths{k}; count = numel(p.t);
    sample_vehicle_id = [sample_vehicle_id;repmat(string(vehicles(k).id),count,1)]; %#ok<AGROW>
    sample_priority = [sample_priority;repmat(vehicles(k).effective_priority,count,1)]; %#ok<AGROW>
    sample_index = [sample_index;(1:count).']; %#ok<AGROW>
    time_s = [time_s;p.t(:)]; x_m = [x_m;p.x(:)]; y_m = [y_m;p.y(:)]; %#ok<AGROW>
    yaw_rad = [yaw_rad;p.theta(:)]; speed_mps = [speed_mps;p.v(:)]; %#ok<AGROW>
    signed_speed_mps = [signed_speed_mps;p.v(:).*p.gear(:)]; %#ok<AGROW>
    acceleration_mps2 = [acceleration_mps2;p.acceleration(:)]; %#ok<AGROW>
    steering_angle_rad = [steering_angle_rad;p.steering_angle(:)]; %#ok<AGROW>
    gear = [gear;p.gear(:)]; %#ok<AGROW>
end
trajectory_table = table(sample_vehicle_id,sample_priority,sample_index,time_s,x_m,y_m, ...
    yaw_rad,speed_mps,signed_speed_mps,acceleration_mps2,steering_angle_rad,gear);
writetable(trajectory_table,files.trajectory_samples_csv,'Encoding','UTF-8');
timings.table_output_s = toc(table_timer);

% 所有车共享同一时间轴；到达车辆按驻车规则保持在终点。
animation_timer = tic;
step = planner.day6.animation_step_s;
frame_rate = planner.day6.animation_frame_rate;
end_time = max(cellfun(@(p)p.t(end),paths));
frame_times = unique([0:step:end_time,end_time]);
obstacles = repmat(struct('id','','states',zeros(0,5),'vehicle_config',struct(), ...
    'footprint',struct(),'hold_end',true),n,1);
poses = cell(n,1);
for k = 1:n
    obstacles(k) = dynamic_vehicle_obstacle(paths{k},vehicle,vehicles(k).id,planner);
    obstacles(k).hold_end = true;
    poses{k} = predict_vehicle_pose(obstacles(k),frame_times);
end

colors = lines(n);
fig = figure('Visible','off','Color','w','Position',[80,80,1000,760]);
cleanup = onCleanup(@()close(fig));
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
plot(ax,geometry.boundary_xy(:,1),geometry.boundary_xy(:,2),'k-', ...
    'LineWidth',1.5,'DisplayName','parking boundary');
for j = 1:numel(geometry.obstacles)
    obstacle = geometry.obstacles{j};
    patch(ax,obstacle(:,1),obstacle(:,2),[.45,.45,.45], ...
        'HandleVisibility','off','DisplayName','static obstacle');
end
xlim(ax,[0,30]); ylim(ax,[0,30]); xlabel(ax,'x / m'); ylabel(ax,'y / m');
path_handles = gobjects(n,1); bodies = gobjects(n,1); rear = gobjects(n,1); headings = gobjects(n,1);
body_vehicle = vehicle;
body_vehicle.collision_margin_m = struct('front',0,'rear',0,'side',0);
for k = 1:n
    p = paths{k};
    path_handles(k) = plot(ax,p.x,p.y,'--','Color',colors(k,:), ...
        'LineWidth',1.15,'DisplayName',[vehicles(k).id ' planned']);
    current = poses{k}(1,:);
    corners = physicalCorners(current,body_vehicle);
    bodies(k) = patch(ax,corners(:,1),corners(:,2),colors(k,:), ...
        'FaceAlpha',.42,'EdgeColor',colors(k,:),'LineWidth',1.3, ...
        'HandleVisibility','off');
    rear(k) = plot(ax,current(1),current(2),'o','Color',colors(k,:), ...
        'MarkerFaceColor',colors(k,:),'HandleVisibility','off');
    headings(k) = quiver(ax,current(1),current(2),cos(current(3)),sin(current(3)),0, ...
        'Color',colors(k,:),'LineWidth',1.4,'HandleVisibility','off');
end
title_handle = title(ax,''); legend(ax,'show','Location','eastoutside','Interpreter','none');
if isprop(ax,'Toolbar'), ax.Toolbar = []; end
disableDefaultInteractivity(ax);
palette = [];
for f = 1:numel(frame_times)
    for k = 1:n
        state = poses{k}(f,:);
        corners = physicalCorners(state,body_vehicle);
        set(bodies(k),'XData',corners(:,1),'YData',corners(:,2));
        set(rear(k),'XData',state(1),'YData',state(2));
        set(headings(k),'XData',state(1),'YData',state(2), ...
            'UData',cos(state(3)),'VData',sin(state(3)));
    end
    title_handle.String = sprintf('Day 7 synchronized four-vehicle animation | t = %.2f s',frame_times(f));
    drawnow limitrate;
    frame = getframe(fig);
    if f == 1
        [indexed,palette] = rgb2ind(frame.cdata,256);
        imwrite(frame.cdata,files.animation_preview_png);
        imwrite(indexed,palette,files.animation_gif,'gif','LoopCount',Inf, ...
            'DelayTime',1/frame_rate);
    else
        indexed = rgb2ind(frame.cdata,palette);
        imwrite(indexed,palette,files.animation_gif,'gif','WriteMode','append', ...
            'DelayTime',1/frame_rate);
    end
end
clear cleanup;
timings.animation_s = toc(animation_timer);
end

function corners = physicalCorners(pose,vehicle)
%PHYSICALCORNERS 根据后轴中心和车身航向变换实际车身四角。
footprint = inflate_vehicle_occupancy(vehicle,0);
rotation = [cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
corners = footprint.corners_local*rotation.'+pose(1:2);
end
