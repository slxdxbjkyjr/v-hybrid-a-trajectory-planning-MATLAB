function summary = benchmark_four_vehicle_lateral(task_file,repetitions,with_profile)
%benchmark_four_vehicle_lateral - Measure fresh searches with a common policy
%   SUMMARY = benchmark_four_vehicle_lateral(task_file) runs one complete
%   four-car search without animation or export inside the measurement.
%   A unique benchmark directory preserves actual configs and failures.
%
%   SUMMARY = benchmark_four_vehicle_lateral(task_file,REPETITIONS) repeats
%   independent searches. Failed runs are retained and stop repetitions.
%   Run once for warmup before collecting at least three timed repetitions.
%
%   SUMMARY = benchmark_four_vehicle_lateral(...,with_profile) enables the
%   MATLAB Profiler. Profiled timings must not be used as speedup claims.
%
%   See also run_four_vehicle_lateral_planning, profile, runtests

if nargin<2, repetitions = 1; end
if nargin<3, with_profile = false; end
validateattributes(repetitions,{'numeric'},{'scalar','integer','positive','finite'});
validateattributes(with_profile,{'logical'},{'scalar'});
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
[~,suffix] = fileparts(tempname);
directory = fullfile(root,'outputs','four_vehicle_lateral_uniform', ...
    ['benchmark_',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')),'_',suffix]);
mkdir(directory);
summary = struct('directory',directory,'profile_enabled',with_profile, ...
    'requested_repetitions',repetitions,'runs',struct([]));
diary(fullfile(directory,'run.log'));
diary_cleanup = onCleanup(@()diary('off'));
writeSummary(fullfile(directory,'input_task.json'),task_file);
for k = 1:repetitions
    if with_profile, profile clear; profile on; end
    result = run_four_vehicle_lateral_planning(task_file,false,fullfile(directory,'progress.json'));
    if with_profile
        profile off;
        profile_data = profile('info');
        save(fullfile(directory,sprintf('run_%02d_profile.mat',k)),'profile_data','-v7');
        functions = profile_data.FunctionTable;
        total = [functions.TotalTime].';
        self = arrayfun(@(f)max(0,f.TotalTime-sum([f.Children.TotalTime])),functions).';
        calls = [functions.NumCalls].';
        names = string({functions.FunctionName}).';
        profile_table = sortrows(table(names(:),total(:),self(:),calls(:), ...
            'VariableNames',{'function_name','total_time_s','self_time_s','calls'}), ...
            'self_time_s','descend');
        writetable(profile_table,fullfile(directory,sprintf('run_%02d_profile.csv',k)));
    end
    stage = result.stages(end);
    records = stage.per_vehicle;
    metrics = table(string({records.id}).',[records.planning_time_s].', ...
        [records.valid].',[records.path_length_m].',[records.arrival_time_s].', ...
        [records.maximum_speed_mps].',[records.peak_lateral_acceleration_mps2].', ...
        'VariableNames',{'vehicle_id','planning_time_s','valid','path_length_m', ...
        'arrival_time_s','maximum_speed_mps','peak_lateral_acceleration_mps2'});
    statistics = {records.search_statistics};
    fields = {'expanded_nodes','generated_nodes','dynamic_pruned','resource_pruned', ...
        'goal_connection_collision_pruned','lateral_acceleration_pruned', ...
        'goal_connection_lateral_acceleration_pruned','closed_pruned_nodes', ...
        'braking_midpoint_pruned','braking_collision_pruned','faster_controls_skipped', ...
        'goal_connection_duplicate_profiles_pruned','final_solution_cost'};
    for field_id = 1:numel(fields)
        name = fields{field_id};
        metrics.(name) = cellfun(@(s)statistic(s,name),statistics(:));
    end
    writetable(metrics,fullfile(directory,sprintf('run_%02d_metrics.csv',k)));
    save(fullfile(directory,sprintf('run_%02d_result.mat',k)),'result','-v7');
    run = struct('valid',result.valid,'reason',result.reason, ...
        'search_time_s',result.search_time_s,'configuration',result.configuration_snapshot, ...
        'metrics',table2struct(metrics),'pairwise_validation',{result.pairwise_validation});
    if k==1
        summary.runs = run;
    else
        summary.runs(k) = run;
    end
    writeSummary(fullfile(directory,'summary.json'),summary);
    fprintf('Run %d: valid=%d, four-car search %.3fs, reason=%s\n', ...
        k,result.valid,result.search_time_s,result.reason);
    if ~result.valid, break; end
end
end

function value = statistic(stats,name)
value = NaN;
if isfield(stats,name), value = stats.(name); end
end

function writeSummary(file,value)
fid = fopen(file,'w','n','UTF-8');
if fid<0, error('Day7Uniform:OutputFailure','Cannot write benchmark summary.'); end
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
