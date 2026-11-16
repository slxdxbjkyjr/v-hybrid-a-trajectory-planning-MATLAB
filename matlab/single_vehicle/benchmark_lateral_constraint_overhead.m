function summary = benchmark_lateral_constraint_overhead(repetitions)
%benchmark_lateral_constraint_overhead - Diagnose lateral constraint overhead
%   SUMMARY = benchmark_lateral_constraint_overhead() searches the current
%   four-car scene twice from scratch, with the common lateral switch on
%   and off. All other inputs and the core search timing scope are equal.
%   Both variants receive independent strict 4.9 m/s^2 lateral diagnostics
%   outside the search timer. These are diagnostic runs, never accepted
%   constrained solutions; this helper does not perform a final pair audit.
%
%   SUMMARY = benchmark_lateral_constraint_overhead(REPETITIONS) repeats
%   both fresh searches. Run a separate warmup before timed repetitions.
%   A unique output directory preserves configs, paths, failures and metrics.
%
%   See also benchmark_four_vehicle_lateral, run_four_vehicle_lateral_planning

if nargin<1, repetitions = 1; end
validateattributes(repetitions,{'numeric'},{'scalar','integer','positive','finite'});
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
config_dir = fullfile(root,'config');
task = jsondecode(fileread(fullfile(config_dir,'four_vehicle_lateral_uniform.json')));
scenario = jsondecode(fileread(fullfile(config_dir,task.scenario_config)));
strict = jsondecode(fileread(fullfile(config_dir,task.planner_config)));
[scenario,strict] = day7_uniform_configuration(scenario,strict);
vehicle = jsondecode(fileread(fullfile(config_dir,task.vehicle_config)));
map_config = jsondecode(fileread(fullfile(config_dir,task.map_config)));
geometry = summon_map(map_config);
strict.vhybrid.minimum_turning_radius_m = vehicle.dimensions_m.wheelbase/ ...
    tan(min(vehicle.steering.maximum_steer_rad,strict.dynamics.delta_max_rad));
validate_st_occupancy_config(strict);
[~,suffix] = fileparts(tempname);
directory = fullfile(root,'outputs','four_vehicle_lateral_uniform', ...
    ['lateral_ablation_',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')),'_',suffix]);
mkdir(directory);
names = {'enabled','disabled'};
fields = {'expanded_nodes','generated_nodes','collision_pruned_nodes', ...
    'constraint_pruned_nodes','closed_pruned_nodes','dynamic_pruned', ...
    'resource_pruned','lateral_acceleration_pruned', ...
    'goal_connection_lateral_acceleration_pruned','goal_connection_collision_pruned', ...
    'braking_collision_pruned','faster_controls_skipped'};
template = struct('variant','','repetition',0,'vehicle_id','','searched',false, ...
    'search_valid',false,'strict_lateral_valid',false,'reason','not_planned', ...
    'search_time_s',NaN,'path_length_m',NaN,'arrival_time_s',NaN, ...
    'maximum_speed_mps',NaN,'strict_peak_abs_ay_mps2',NaN);
for k=1:numel(fields), template.(fields{k}) = NaN; end
rows = repmat(template,8*repetitions,1);
runs = cell(2*repetitions,1);
for repeat=1:repetitions
    for variant=1:2
        planner = strict;
        planner.dynamics.lateral_acceleration_constraint_enabled = variant==1;
        resource = planner;
        resource.st_occupancy.safety_distance_m = max(resource.st_occupancy.safety_distance_m, ...
            planner.day6.minimum_safety_distance_m/2);
        resource.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
        st_map = st_occupancy_map(resource,geometry);
        st_map = mark_static_obstacles(st_map,geometry,vehicle);
        obstacles = struct('id',{},'states',{},'vehicle_config',{},'footprint',{},'hold_end',{});
        paths = cell(4,1); strict_reports = cell(4,1);
        run_index = (repeat-1)*2+variant;
        offset = (run_index-1)*4;
        for k=1:4
            rows(offset+k).variant = names{variant};
            rows(offset+k).repetition = repeat;
            rows(offset+k).vehicle_id = scenario.planning_order{k};
        end
        for k=1:4
            id = scenario.planning_order{k};
            car = scenario.vehicles(strcmp({scenario.vehicles.id},id));
            a = car.start_pose; b = car.summon_goal;
            start = [a.x_m,a.y_m,a.yaw_rad,0,0]; goal = [b.x_m,b.y_m,b.yaw_rad,0,0];
            timer = tic;
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,st_map,obstacles);
            elapsed = toc(timer);
            strict_reports{k} = vhybrid_validate_lateral_acceleration(path,vehicle,strict);
            if isfield(path,'search_nodes'), path.search_nodes = path.search_nodes([]); end
            paths{k} = path; row = rows(offset+k);
            row.searched = true; row.search_valid = path.valid; row.search_time_s = elapsed;
            row.strict_lateral_valid = strict_reports{k}.valid;
            row.strict_peak_abs_ay_mps2 = strict_reports{k}.max_peak_abs_ay_mps2;
            row.reason = 'search_failed';
            if isfield(path.failure_diagnostics,'reason')
                row.reason = path.failure_diagnostics.reason;
            end
            for j=1:numel(fields), row.(fields{j}) = path.search_statistics.(fields{j}); end
            if path.valid
                row.reason = 'search_returned_path_diagnostic_only';
                row.path_length_m = sum(abs(path.travelled_distance));
                row.arrival_time_s = path.t(end); row.maximum_speed_mps = max(abs(path.v));
            end
            rows(offset+k) = row;
            fprintf('%s repeat%d %s: valid=%d, search=%.3fs, strict_ay=%.6f\n', ...
                names{variant},repeat,id,path.valid,elapsed,row.strict_peak_abs_ay_mps2);
            if ~path.valid, break; end
            st_map = mark_trajectory_occupancy(st_map,path,vehicle,id);
            obstacles(end+1,1) = dynamic_vehicle_obstacle(path,vehicle,id,planner); %#ok<AGROW>
            st_map = prepare_dynamic_context(st_map,obstacles);
        end
        records = rows(offset+(1:4));
        file = sprintf('%s_repeat_%02d.mat',names{variant},repeat);
        diagnostic_only = true;
        save(fullfile(directory,file),'diagnostic_only','task','scenario','planner', ...
            'strict','vehicle','map_config','paths','strict_reports','records','-v7');
        runs{run_index} = struct('variant',names{variant},'repetition',repeat, ...
            'all_searches_valid',all([records.search_valid]), ...
            'all_strict_lateral_valid',all([records.strict_lateral_valid]), ...
            'core_search_time_s',sum([records.search_time_s],'omitnan'),'result_mat',file);
        writetable(struct2table(rows(1:offset+4)),fullfile(directory,'vehicle_metrics.csv'));
    end
end
summary = struct('diagnostic_only',true,'directory',directory,'repetitions',repetitions, ...
    'timing_scope','core search calls only; strict lateral audit and occupancy updates excluded', ...
    'changed_field','dynamics.lateral_acceleration_constraint_enabled', ...
    'pairwise_audit_performed',false,'strict_configuration',strict,'runs',{runs});
fid = fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');
if fid<0, error('Day7Uniform:OutputFailure','Cannot create lateral ablation summary.'); end
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
end
