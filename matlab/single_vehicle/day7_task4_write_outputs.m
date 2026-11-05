function [files, metrics] = day7_task4_write_outputs(stage_result, vehicles, geometry, vehicle, planner, output_dir, tag)
%DAY7_TASK4_WRITE_OUTPUTS Write isolated Task 4 artifacts for a stage result.
% paths and vehicles are aligned; missing or invalid paths remain in the
% metrics/report and do not prevent outputs for other successful vehicles.

if nargin < 7 || isempty(tag), tag = 'stage'; end
if ~(ischar(tag) && isrow(tag)) && ~(isstring(tag) && isscalar(tag))
    error('Day7Task4:InvalidTag','tag must be a character row or string scalar.');
end
tag = regexprep(char(tag),'[^A-Za-z0-9_-]','_');
if isempty(tag), tag = 'stage'; end
if ~isstruct(stage_result) || ~isscalar(stage_result) || ...
        ~isstruct(vehicles) || ~isvector(vehicles) || ~iscell(stage_result.paths) || ...
        numel(stage_result.paths) ~= numel(vehicles)
    error('Day7Task4:InvalidInputs', ...
        'stage_result.paths must be a cell array aligned with vehicles.');
end
if ~exist(output_dir,'dir'), mkdir(output_dir); end

n = numel(vehicles);
paths = stage_result.paths(:);
vehicle_ids = strings(n,1); priority = nan(n,1); status = strings(n,1);
path_valid = false(n,1); kinematic_valid = false(n,1);
lateral_enabled = false(n,1); lateral_valid = false(n,1);
path_length_m = nan(n,1); arrival_time_s = nan(n,1); max_speed_mps = nan(n,1);
peak_abs_ay_mps2 = nan(n,1); accel_min_mps2 = nan(n,1); accel_max_mps2 = nan(n,1);
planning_time_s = nan(n,1); terminal_speed_mps = nan(n,1);
expanded_nodes = nan(n,1); generated_nodes = nan(n,1);
dynamic_pruned = nan(n,1); resource_pruned = nan(n,1);
lateral_pruned = nan(n,1); connection_lateral_pruned = nan(n,1);
collision_pruned = nan(n,1); constraint_pruned = nan(n,1); closed_pruned = nan(n,1);
connection_collision_pruned = nan(n,1); connection_cost_pruned = nan(n,1);
connection_boundary_pruned = nan(n,1); connection_turning_pruned = nan(n,1);
braking_midpoint_pruned = nan(n,1); horizon_pruned = nan(n,1); gear_switch_pruned = nan(n,1);
validation_reports = cell(n,1); lateral_reports = cell(n,1);
valid_indices = zeros(0,1);

for k = 1:n
    record = vehicles(k);
    vehicle_ids(k) = string(textField(record,'id',sprintf('vehicle_%03d',k)));
    priority(k) = numericField(record,'effective_priority',numericField(record,'priority',NaN));
    status(k) = string(textField(record,'reason','not_planned'));
    planning_time_s(k) = numericField(record,'planning_time_s',NaN);
    stats = structField(record,'search_statistics',struct());
    expanded_nodes(k) = numericField(stats,'expanded_nodes',NaN);
    generated_nodes(k) = numericField(stats,'generated_nodes',NaN);
    dynamic_pruned(k) = numericField(stats,'dynamic_pruned',NaN);
    resource_pruned(k) = numericField(stats,'resource_pruned',NaN);
    collision_pruned(k) = numericField(stats,'collision_pruned_nodes',NaN);
    constraint_pruned(k) = numericField(stats,'constraint_pruned_nodes',NaN);
    closed_pruned(k) = numericField(stats,'closed_pruned_nodes',NaN);
    lateral_pruned(k) = numericField(stats,'lateral_acceleration_pruned',NaN);
    connection_lateral_pruned(k) = numericField(stats, ...
        'goal_connection_lateral_acceleration_pruned',NaN);
    connection_collision_pruned(k) = numericField(stats,'goal_connection_collision_pruned',NaN);
    connection_cost_pruned(k) = numericField(stats,'goal_connection_cost_bound_pruned',NaN);
    connection_boundary_pruned(k) = numericField(stats,'goal_connection_boundary_pruned',NaN);
    connection_turning_pruned(k) = numericField(stats,'goal_connection_turning_pruned',NaN);
    braking_midpoint_pruned(k) = numericField(stats,'braking_midpoint_pruned',NaN);
    horizon_pruned(k) = numericField(stats,'horizon_pruned',NaN);
    gear_switch_pruned(k) = numericField(stats,'gear_switch_pruned',NaN);

    p = paths{k};
    if ~isstruct(p) || ~isscalar(p) || ~isfield(p,'valid') || ~p.valid || ...
            ~all(isfield(p,{'x','y','theta','v','t','acceleration','steering_angle'}))
        if isfield(record,'valid') && ~record.valid && strlength(status(k)) == 0
            status(k) = "search_failed";
        elseif strlength(status(k)) == 0
            status(k) = "path_missing_or_invalid";
        end
        validation_reports{k} = struct('valid',false,'failure_reasons',{{'path_missing_or_invalid'}});
        lateral_reports{k} = struct('enabled',lateralConstraintEnabled(planner), ...
            'valid',false,'reason','path_missing_or_invalid','max_peak_abs_ay_mps2',NaN, ...
            'segments',repmat(struct(),0,1));
        lateral_enabled(k) = lateralConstraintEnabled(planner);
        continue;
    end
    try
        ensurePathShape(p,vehicle_ids(k));
        valid_indices(end+1,1) = k; %#ok<AGROW>
        path_valid(k) = true;
        path_length_m(k) = pathLength(p);
        arrival_time_s(k) = double(p.t(end));
        terminal_speed_mps(k) = double(p.v(end));
        max_speed_mps(k) = max(abs(double(p.v(:))));
        accel_min_mps2(k) = min(double(p.acceleration(:)));
        accel_max_mps2(k) = max(double(p.acceleration(:)));
        replay = vhybrid_validate_trajectory(p,geometry,vehicle,planner);
        lateral = vhybrid_validate_lateral_acceleration(p,vehicle,planner);
        validation_reports{k} = replay;
        lateral_reports{k} = lateral;
        kinematic_valid(k) = logical(replay.valid);
        lateral_enabled(k) = logical(lateral.enabled);
        [peak_abs_ay_mps2(k),lateral_valid(k),lateral] = lateralSummary(p,lateral,vehicle,planner);
        lateral_reports{k} = lateral;
        validated_status = string(validationStatus(replay,lateral));
        if validated_status ~= "passed" || strlength(status(k)) == 0 || status(k) == "not_planned"
            status(k) = validated_status;
        end
    catch exception
        path_valid(k) = false;
        valid_indices(valid_indices == k) = [];
        validation_reports{k} = struct('valid',false,'failure_reasons',{{exception.identifier,exception.message}});
        lateral_reports{k} = struct('enabled',lateralConstraintEnabled(planner), ...
            'valid',false,'reason','validation_error','failure_message',exception.message, ...
            'max_peak_abs_ay_mps2',NaN,'segments',repmat(struct(),0,1));
        lateral_enabled(k) = lateralConstraintEnabled(planner);
        status(k) = "output_validation_error: " + string(exception.message);
    end
end

metrics = table(vehicle_ids,priority,status,path_valid,kinematic_valid,lateral_enabled, ...
    lateral_valid,path_length_m,arrival_time_s,max_speed_mps,peak_abs_ay_mps2, ...
    accel_min_mps2,accel_max_mps2,planning_time_s,terminal_speed_mps,expanded_nodes, ...
    generated_nodes,collision_pruned,constraint_pruned,closed_pruned,dynamic_pruned, ...
    resource_pruned,lateral_pruned,connection_lateral_pruned,connection_collision_pruned, ...
    connection_cost_pruned,connection_boundary_pruned,connection_turning_pruned, ...
    braking_midpoint_pruned,horizon_pruned,gear_switch_pruned, ...
    'VariableNames',{'vehicle_id','priority','status','path_valid','kinematic_valid', ...
    'lateral_constraint_enabled','lateral_valid','path_length_m','arrival_time_s', ...
    'max_speed_mps','peak_abs_ay_mps2','longitudinal_acceleration_min_mps2', ...
    'longitudinal_acceleration_max_mps2','planning_time_s','terminal_speed_mps', ...
    'expanded_nodes','generated_nodes','collision_pruned_nodes','constraint_pruned_nodes', ...
    'closed_pruned_nodes','dynamic_pruned','resource_pruned','lateral_acceleration_pruned', ...
    'goal_connection_lateral_acceleration_pruned','goal_connection_collision_pruned', ...
    'goal_connection_cost_bound_pruned','goal_connection_boundary_pruned', ...
    'goal_connection_turning_pruned','braking_midpoint_pruned','horizon_pruned', ...
    'gear_switch_pruned'});

prefix = fullfile(output_dir,[tag '_']);
files = struct('metrics_csv',[prefix 'time_metrics.csv'], ...
    'trajectory_csv',[prefix 'trajectory_samples.csv'], ...
    'xy_plot',[prefix 'xy.png'],'speed_plot',[prefix 'speed.png'], ...
    'lateral_acceleration_plot',[prefix 'lateral_acceleration.png'], ...
    'animation_gif',[prefix 'animation.gif'],'animation_preview',[prefix 'animation_preview.png'], ...
    'mat',[prefix 'result.mat'],'json',[prefix 'result.json']);
writetable(metrics,files.metrics_csv,'Encoding','UTF-8');

samples = makeTrajectorySamples(paths,valid_indices,vehicle_ids,priority, ...
    lateral_reports,vehicle);
writetable(samples,files.trajectory_csv,'Encoding','UTF-8');
makePlots(paths,valid_indices,vehicle_ids,geometry,planner,lateral_reports,files,tag);
writeAnimation(paths,valid_indices,vehicle_ids,geometry,vehicle,planner,files,tag);

report = struct('tag',tag,'stage',textField(stage_result,'stage',''), ...
    'valid',logicalField(stage_result,'valid',false), ...
    'reason',textField(stage_result,'reason',''), ...
    'failure',structField(stage_result,'failure',struct()), ...
    'planning_order',{cellstr(vehicle_ids)},'metrics',table2struct(metrics), ...
    'search_statistics',{cell(n,1)},'kinematic_validation',{validation_reports}, ...
    'lateral_acceleration_validation',{lateral_reports}, ...
    'pairwise_validation',structField(stage_result,'pairwise_validation',struct([])), ...
    'output_files',files);
for k = 1:n
    report.search_statistics{k} = structField(vehicles(k),'search_statistics',struct());
end
fid = fopen(files.json,'w','n','UTF-8');
if fid < 0, error('Day7Task4:OutputFailure','Cannot create %s.',files.json); end
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
clear cleanup;

saved_paths = paths;
for k = 1:numel(saved_paths)
    if isstruct(saved_paths{k}) && isfield(saved_paths{k},'search_nodes')
        saved_paths{k}.search_nodes = saved_paths{k}.search_nodes([]);
    end
end
save(files.mat,'saved_paths','metrics','report','-v7');
end

function ensurePathShape(path,id)
fields = {'x','y','theta','v','t','acceleration','steering_angle'};
n = numel(path.t);
if n < 2 || any(~isfinite(double(path.t(:)))) || any(diff(double(path.t(:))) <= 0)
    error('Day7Task4:InvalidPath','%s has invalid or nonincreasing timestamps.',id);
end
for k = 1:numel(fields)
    value = path.(fields{k});
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || numel(value) ~= n || ...
            any(~isfinite(double(value(:))))
        error('Day7Task4:InvalidPath','%s field %s is invalid.',id,fields{k});
    end
end
if isfield(path,'gear') && (~isnumeric(path.gear) || numel(path.gear) ~= n || ...
        any(~ismember(path.gear(:),[-1,1])))
    error('Day7Task4:InvalidPath','%s has invalid gear samples.',id);
end
end

function length_m = pathLength(path)
if isfield(path,'travelled_distance') && isnumeric(path.travelled_distance) && ...
        numel(path.travelled_distance) == numel(path.t)
    length_m = sum(abs(double(path.travelled_distance(:))));
else
    length_m = sum(hypot(diff(double(path.x(:))),diff(double(path.y(:)))));
end
end

function [peak,valid,report] = lateralSummary(path,report,vehicle,planner)
enabled = lateralConstraintEnabled(planner);
if isfield(report,'enabled') && report.enabled
    valid = logical(report.valid);
    peak = double(report.max_peak_abs_ay_mps2);
    return;
end
wheelbase = double(vehicle.dimensions_m.wheelbase);
delta = double(path.steering_angle(:));
v0 = double(path.v(:));
dt = diff(double(path.t(:)));
acc = double(path.acceleration(:));
curvature = tan(delta(2:end))/wheelbase;
v_model_end = v0(1:end-1)+acc(2:end).*dt;
peak_by_segment = max(abs([v0(1:end-1),v0(2:end),v_model_end]),[],2).^2.*abs(curvature);
peak = max([0;peak_by_segment]);
limit = Inf;
if enabled
    cfg = validate_lateral_acceleration_config(planner.dynamics);
    limit = cfg.max_lateral_acceleration_mps2+cfg.lateral_acceleration_tolerance_mps2;
end
valid = all(isfinite(peak_by_segment)) && all(peak_by_segment <= limit);
segments = repmat(struct('time_start_s',0,'time_end_s',0,'curvature_per_m',0, ...
    'peak_abs_ay_mps2',0,'valid',true,'reason',''),numel(dt),1);
for k = 1:numel(dt)
    segments(k) = struct('time_start_s',double(path.t(k)), ...
        'time_end_s',double(path.t(k+1)),'curvature_per_m',curvature(k), ...
        'peak_abs_ay_mps2',peak_by_segment(k),'valid',peak_by_segment(k)<=limit, ...
        'reason',conditional(peak_by_segment(k)<=limit,'passed','lateral_acceleration_exceeded'));
end
report = struct('enabled',enabled,'valid',valid,'reason',conditional(valid,'passed', ...
    'lateral_acceleration_exceeded'),'max_peak_abs_ay_mps2',peak,'segments',segments);
end

function samples = makeTrajectorySamples(paths,indices,ids,priority,lateral_reports,vehicle)
vehicle_id = strings(0,1); sample_priority = zeros(0,1); sample_index = zeros(0,1); time_s = zeros(0,1);
x_m = zeros(0,1); y_m = zeros(0,1); yaw_rad = zeros(0,1); speed_mps = zeros(0,1);
signed_speed_mps = zeros(0,1); acceleration_mps2 = zeros(0,1); steering_angle_rad = zeros(0,1);
gear = zeros(0,1); curvature_per_m = zeros(0,1); lateral_acceleration_peak_mps2 = zeros(0,1);
for index = indices(:).'
    p = paths{index}; n = numel(p.t);
    path_gear = ones(n,1);
    if isfield(p,'gear'), path_gear = double(p.gear(:)); end
    curv = zeros(n,1); ay = zeros(n,1);
    detail = lateral_reports{index};
    if isfield(detail,'segments') && numel(detail.segments) == n-1
        curv(2:end) = [detail.segments.curvature_per_m].';
        ay(2:end) = [detail.segments.peak_abs_ay_mps2].';
    else
        curv(2:end) = tan(double(p.steering_angle(2:end)))/double(vehicle.dimensions_m.wheelbase);
        dt = diff(double(p.t(:)));
        model_end = double(p.v(1:end-1))+double(p.acceleration(2:end)).*dt;
        ay(2:end) = max(abs([double(p.v(1:end-1)),double(p.v(2:end)),model_end]),[],2).^2.*abs(curv(2:end));
    end
    vehicle_id = [vehicle_id;repmat(ids(index),n,1)]; %#ok<AGROW>
    sample_priority = [sample_priority;repmat(priority(index),n,1)]; %#ok<AGROW>
    sample_index = [sample_index;(1:n).']; time_s = [time_s;double(p.t(:))]; %#ok<AGROW>
    x_m = [x_m;double(p.x(:))]; y_m = [y_m;double(p.y(:))]; yaw_rad = [yaw_rad;double(p.theta(:))]; %#ok<AGROW>
    speed_mps = [speed_mps;double(p.v(:))]; signed_speed_mps = [signed_speed_mps;double(p.v(:)).*path_gear]; %#ok<AGROW>
    acceleration_mps2 = [acceleration_mps2;double(p.acceleration(:))]; %#ok<AGROW>
    steering_angle_rad = [steering_angle_rad;double(p.steering_angle(:))]; gear = [gear;path_gear]; %#ok<AGROW>
    curvature_per_m = [curvature_per_m;curv]; lateral_acceleration_peak_mps2 = [lateral_acceleration_peak_mps2;ay]; %#ok<AGROW>
end
samples = table(vehicle_id,sample_priority,sample_index,time_s,x_m,y_m,yaw_rad,speed_mps,signed_speed_mps, ...
    acceleration_mps2,steering_angle_rad,gear,curvature_per_m,lateral_acceleration_peak_mps2);
end

function makePlots(paths,indices,ids,geometry,planner,lateral_reports,files,tag)
colors = lines(max(1,numel(paths)));
fig = figure('Visible','off','Color','w','Position',[80,80,1050,760]);
cleanup = onCleanup(@()close(fig));
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
if isfield(geometry,'boundary_xy')
    plot(ax,geometry.boundary_xy(:,1),geometry.boundary_xy(:,2),'k-','LineWidth',1.4,'DisplayName','boundary');
end
if isfield(geometry,'obstacles')
    for k = 1:numel(geometry.obstacles)
        poly = geometry.obstacles{k};
        patch(ax,poly(:,1),poly(:,2),[.5,.5,.5],'HandleVisibility','off');
    end
end
for index = indices(:).'
    p = paths{index};
    plot(ax,p.x,p.y,'-','Color',colors(index,:),'LineWidth',1.5,'DisplayName',char(ids(index)));
    plot(ax,p.x(1),p.y(1),'o','Color',colors(index,:),'MarkerFaceColor',colors(index,:),'HandleVisibility','off');
    plot(ax,p.x(end),p.y(end),'s','Color',colors(index,:),'MarkerFaceColor','w','HandleVisibility','off');
end
if isempty(indices), text(ax,.5,.5,'No valid paths in this stage','Units','normalized','HorizontalAlignment','center'); end
if isfield(geometry,'bounds') && numel(geometry.bounds)==4
    xlim(ax,geometry.bounds([1,3])); ylim(ax,geometry.bounds([2,4]));
end
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); title(ax,['Task 4 XY paths: ' tag],'Interpreter','none');
legend(ax,'show','Location','best','Interpreter','none'); saveas(fig,files.xy_plot);
clf(fig); ax = axes(fig); hold(ax,'on'); grid(ax,'on');
for index = indices(:).'
    p = paths{index}; gear = ones(numel(p.t),1);
    if isfield(p,'gear'), gear = double(p.gear(:)); end
    plot(ax,p.t,double(p.v(:)).*gear,'Color',colors(index,:),'LineWidth',1.4,'DisplayName',char(ids(index)));
end
if isempty(indices), text(ax,.5,.5,'No valid paths in this stage','Units','normalized','HorizontalAlignment','center'); end
xlabel(ax,'time (s)'); ylabel(ax,'signed speed (m/s)'); title(ax,['Task 4 speed: ' tag],'Interpreter','none');
legend(ax,'show','Location','best','Interpreter','none'); saveas(fig,files.speed_plot);
clf(fig); ax = axes(fig); hold(ax,'on'); grid(ax,'on');
for index = indices(:).'
    seg = lateral_reports{index}.segments;
    if isempty(seg), continue; end
    t = ([seg.time_start_s]+[seg.time_end_s])/2;
    plot(ax,t,[seg.peak_abs_ay_mps2],'Color',colors(index,:),'LineWidth',1.2,'DisplayName',char(ids(index)));
end
limit = 4.9;
if isfield(planner,'dynamics') && lateralConstraintEnabled(planner)
    cfg = validate_lateral_acceleration_config(planner.dynamics);
    limit = cfg.max_lateral_acceleration_mps2;
end
yline(ax,limit,'r--','ay limit','HandleVisibility','off');
if isempty(indices), text(ax,.5,.5,'No valid paths in this stage','Units','normalized','HorizontalAlignment','center'); end
xlabel(ax,'time (s)'); ylabel(ax,'peak |a_y| per segment (m/s^2)');
title(ax,['Task 4 lateral acceleration: ' tag],'Interpreter','none');
legend(ax,'show','Location','best','Interpreter','none'); saveas(fig,files.lateral_acceleration_plot);
clear cleanup;
end

function writeAnimation(paths,indices,ids,geometry,vehicle,planner,files,tag)
step = .2; frame_rate = 10;
if isfield(planner,'day6')
    if isfield(planner.day6,'animation_step_s'), step = planner.day6.animation_step_s; end
    if isfield(planner.day6,'animation_frame_rate'), frame_rate = planner.day6.animation_frame_rate; end
end
valid_paths = paths(indices);
if isempty(valid_paths)
    times = 0;
else
    end_time = max(cellfun(@(p)double(p.t(end)),valid_paths));
    times = unique([0:step:end_time,end_time]);
end
fig = figure('Visible','off','Color','w','Position',[80,80,1000,760]);
cleanup = onCleanup(@()close(fig));
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
if isfield(geometry,'boundary_xy')
    plot(ax,geometry.boundary_xy(:,1),geometry.boundary_xy(:,2),'k-','LineWidth',1.4);
end
if isfield(geometry,'obstacles')
    for k = 1:numel(geometry.obstacles)
        poly = geometry.obstacles{k}; patch(ax,poly(:,1),poly(:,2),[.5,.5,.5],'HandleVisibility','off');
    end
end
if isfield(geometry,'bounds') && numel(geometry.bounds)==4
    xlim(ax,geometry.bounds([1,3])); ylim(ax,geometry.bounds([2,4]));
end
xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
colors = lines(max(1,numel(paths))); body = vehicle;
if isfield(body,'collision_margin_m'), body.collision_margin_m = struct('front',0,'rear',0,'side',0); end
poses = cell(numel(indices),1); body_handles = gobjects(numel(indices),1);
rear_handles = gobjects(numel(indices),1); heading_handles = gobjects(numel(indices),1);
for j = 1:numel(indices)
    index = indices(j); p = paths{index};
    obstacle = dynamic_vehicle_obstacle(p,vehicle,char(ids(index)),planner);
    obstacle.hold_end = true;
    poses{j} = predict_vehicle_pose(obstacle,times);
    plot(ax,p.x,p.y,'--','Color',colors(index,:),'DisplayName',char(ids(index)));
    corners = physicalCorners(poses{j}(1,:),body);
    body_handles(j) = patch(ax,corners(:,1),corners(:,2),colors(index,:), ...
        'FaceAlpha',.38,'EdgeColor',colors(index,:),'HandleVisibility','off');
    rear_handles(j) = plot(ax,poses{j}(1,1),poses{j}(1,2),'o','Color',colors(index,:), ...
        'MarkerFaceColor',colors(index,:),'HandleVisibility','off');
    heading_handles(j) = quiver(ax,poses{j}(1,1),poses{j}(1,2), ...
        cos(poses{j}(1,3)),sin(poses{j}(1,3)),0,'Color',colors(index,:), ...
        'HandleVisibility','off');
end
if isempty(indices)
    text(ax,.5,.5,'No valid paths in this stage','Units','normalized','HorizontalAlignment','center');
end
legend(ax,'show','Location','best','Interpreter','none'); title_handle = title(ax,'');
palette = [];
for f = 1:numel(times)
    for j = 1:numel(indices)
        pose = poses{j}(f,:); corners = physicalCorners(pose,body);
        set(body_handles(j),'XData',corners(:,1),'YData',corners(:,2));
        set(rear_handles(j),'XData',pose(1),'YData',pose(2));
        set(heading_handles(j),'XData',pose(1),'YData',pose(2), ...
            'UData',cos(pose(3)),'VData',sin(pose(3)));
    end
    title_handle.String = sprintf('Task 4 %s | t = %.2f s',tag,times(f));
    drawnow limitrate; frame = getframe(fig);
    if f == 1
        [indexed,palette] = rgb2ind(frame.cdata,256);
        imwrite(frame.cdata,files.animation_preview);
        imwrite(indexed,palette,files.animation_gif,'gif','LoopCount',Inf,'DelayTime',1/frame_rate);
    else
        indexed = rgb2ind(frame.cdata,palette);
        imwrite(indexed,palette,files.animation_gif,'gif','WriteMode','append','DelayTime',1/frame_rate);
    end
end
clear cleanup;
end

function corners = physicalCorners(pose,vehicle)
footprint = inflate_vehicle_occupancy(vehicle,0);
rotation = [cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
corners = footprint.corners_local*rotation.'+pose(1:2);
end

function enabled = lateralConstraintEnabled(planner)
enabled = false;
if isstruct(planner) && isfield(planner,'dynamics') && ...
        isfield(planner.dynamics,'lateral_acceleration_constraint_enabled')
    enabled = logical(planner.dynamics.lateral_acceleration_constraint_enabled);
end
end

function value = textField(s,name,default)
value = default;
if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
    value = s.(name);
end
end

function value = numericField(s,name,default)
value = default;
if isstruct(s) && isfield(s,name) && isnumeric(s.(name)) && isscalar(s.(name))
    value = double(s.(name));
end
end

function value = logicalField(s,name,default)
value = default;
if isstruct(s) && isfield(s,name) && islogical(s.(name)) && isscalar(s.(name))
    value = s.(name);
end
end

function value = structField(s,name,default)
value = default;
if isstruct(s) && isfield(s,name) && isstruct(s.(name))
    value = s.(name);
end
end

function value = validationStatus(replay,lateral)
if ~replay.valid
    value = 'kinematic_validation_failed';
elseif isfield(lateral,'enabled') && lateral.enabled && ~lateral.valid
    value = 'lateral_acceleration_validation_failed';
else
    value = 'passed';
end
end

function out = conditional(condition,yes,no)
if condition, out = yes; else, out = no; end
end
