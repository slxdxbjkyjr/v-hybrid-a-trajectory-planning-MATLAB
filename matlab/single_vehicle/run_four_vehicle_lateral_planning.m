function result = run_four_vehicle_lateral_planning(task_file,save_outputs,progress_file)
%run_four_vehicle_lateral_planning - Search a new four-car scene from rest
%   RESULT = run_four_vehicle_lateral_planning() loads the independent
%   uniform task configuration. All cars use exactly one search policy.
%
%   RESULT = run_four_vehicle_lateral_planning(task_file) reads a task JSON
%   or scalar struct referencing scenario, planner, vehicle and map JSONs.
%   A struct may supply SCENARIO directly for tests. Historical trajectories
%   and per-vehicle overrides are not accepted as planning inputs.
%
%   RESULT = run_four_vehicle_lateral_planning(task_file,save_outputs)
%   disables all file and figure outputs when save_outputs is false. True
%   creates a unique run directory, preserving previous measurements.
%
%   RESULT = run_four_vehicle_lateral_planning(...,progress_file) writes
%   per-car progress to the supplied JSON path even with outputs disabled.
%
%   See also day7_uniform_configuration, run_day7_lateral_limit_task4

project_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1 || isempty(task_file), task_file = 'four_vehicle_lateral_uniform.json'; end
if nargin<2, save_outputs = true; end
if nargin<3, progress_file = ''; end
if ~islogical(save_outputs) || ~isscalar(save_outputs)
    error('Day7Uniform:InvalidOutputOption','save_outputs must be logical.');
end
if isstruct(task_file)
    task = task_file;
else
    source = char(task_file);
    if ~isfile(source), source = fullfile(project_dir,'config',source); end
    task = jsondecode(fileread(source));
end
allowed = {'schema_version','description','scenario_config','scenario', ...
    'planner_config','vehicle_config','map_config','output_directory'};
if ~isstruct(task) || ~isscalar(task) || ~isempty(setdiff(fieldnames(task),allowed))
    error('Day7Uniform:TaskOverrides','Only scene/config references and output location are accepted.');
end
if ~all(isfield(task,{'planner_config','vehicle_config','map_config','output_directory'})) || ...
        (~isfield(task,'scenario_config') && ~isfield(task,'scenario'))
    error('Day7Uniform:IncompleteTask','Provide the scenario and common configuration files.');
end
task.planning_policy = 'uniform_shared';
if save_outputs
    % The task output root must be separate from frozen regression artifacts.
    normalized = strrep(char(task.output_directory),'\','/');
    if ~strcmp(normalized,'outputs/four_vehicle_lateral_uniform')
        error('Day7Uniform:OutputRoot','Use outputs/four_vehicle_lateral_uniform.');
    end
    [~,suffix] = fileparts(tempname);
    task.output_directory = fullfile(normalized, ...
        [char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')),'_',suffix]);
end
options = struct('save_outputs',save_outputs);
if ~isempty(progress_file), options.progress_file = progress_file; end
result = run_day7_lateral_limit_task4(task,options);
end
