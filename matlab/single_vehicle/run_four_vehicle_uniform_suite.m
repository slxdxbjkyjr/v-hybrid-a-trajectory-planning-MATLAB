function summary = run_four_vehicle_uniform_suite(save_outputs)
%run_four_vehicle_uniform_suite - Validate the six agreed four-car scenarios
%   SUMMARY = run_four_vehicle_uniform_suite() searches each configured
%   scene anew with the same frozen planner and saves separate artifacts.
%
%   SUMMARY = run_four_vehicle_uniform_suite(save_outputs) suppresses scene
%   plots and animations when false. The suite summary is always written
%   under a unique directory and includes failures as well as successes.
%
%   See also run_four_vehicle_lateral_planning, benchmark_four_vehicle_lateral

if nargin<1, save_outputs = true; end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
manifest = jsondecode(fileread(fullfile(root,'config','four_vehicle_uniform_suite.json')));
task = jsondecode(fileread(fullfile(root,'config','four_vehicle_lateral_uniform.json')));
[~,suffix] = fileparts(tempname);
directory = fullfile(root,'outputs','four_vehicle_lateral_uniform', ...
    ['suite_',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')),'_',suffix]);
mkdir(directory);
rows = repmat(struct('id','','valid',false,'reason','','search_time_s',NaN, ...
    'output_index','','pairwise_valid',false,'effective_config_identical',false), ...
    numel(manifest.scenarios),1);
common = [];
for k = 1:numel(manifest.scenarios)
    task.scenario_config = manifest.scenarios(k).scenario_config;
    result = run_four_vehicle_lateral_planning(task,save_outputs);
    config = result.effective_configuration.common_search_config;
    if isempty(common), common = config; end
    rows(k).id = manifest.scenarios(k).id;
    rows(k).valid = result.valid;
    rows(k).reason = result.reason;
    rows(k).search_time_s = result.search_time_s;
    rows(k).effective_config_identical = isequal(common,config);
    rows(k).pairwise_valid = numel(result.pairwise_validation)==6 && ...
        all([result.pairwise_validation.valid]);
    if isfield(result.output_files,'output_index')
        rows(k).output_index = result.output_files.output_index;
    end
    save(fullfile(directory,[rows(k).id,'_result.mat']),'result','-v7');
    writetable(struct2table(rows(1:k),'AsArray',true), ...
        fullfile(directory,'scenario_metrics.csv'));
    fprintf('Scene %s: %s, %.3fs\n',rows(k).id,rows(k).reason,rows(k).search_time_s);
end
summary = struct('directory',directory,'scenario_count',numel(rows), ...
    'passed',nnz([rows.valid]),'failed',nnz(~[rows.valid]), ...
    'common_search_config',common,'scenarios',rows);
fid = fopen(fullfile(directory,'suite_summary.json'),'w','n','UTF-8');
if fid<0, error('Day7Uniform:OutputFailure','Cannot create suite summary.'); end
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
end
