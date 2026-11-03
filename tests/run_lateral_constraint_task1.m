function report = run_lateral_constraint_task1(source_root,output_directory)
%RUN_LATERAL_CONSTRAINT_TASK1 Run task-one tests without running a new search.
% Source code/tests come from this project. source_root supplies the original
% saved Day 7 MAT for read-only diagnosis. Original outputs are never written.
project = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, source_root = project; end
if nargin < 2
    output_directory = fullfile(project,'outputs','day7_lateral_constraint','task1');
end
output_directory = char(java.io.File(char(output_directory)).getCanonicalPath());
allowed = char(java.io.File(fullfile(project,'outputs', ...
    'day7_lateral_constraint')).getCanonicalPath());
if ~strcmpi(output_directory,allowed) && ...
        ~startsWith(lower(output_directory),lower([allowed,filesep]))
    error('LateralAcceleration:InvalidOutputDirectory', ...
        'Task-one outputs must be inside outputs/day7_lateral_constraint.');
end
previous_path = path;
addpath(fullfile(project,'matlab','single_vehicle'));
fixture_cleanup = onCleanup(@() path(previous_path)); %#ok<NASGU>
test_dir = fullfile(project,'tests');
names = {'test_vehicle_dynamic.m','test_reverse_motion.m','test_reverse_initial_gear.m', ...
    'test_lateral_acceleration.m','test_day7_scenario.m'};
suite = matlab.unittest.Test.empty(1,0);
for k = 1:numel(names)
    suite = [suite,matlab.unittest.TestSuite.fromFile(fullfile(test_dir,names{k}))]; %#ok<AGROW>
end
results = run(suite);
test_records = repmat(struct('name','','passed',false,'failed',false, ...
    'incomplete',false,'duration_s',0,'diagnostics',''),numel(results),1);
for k = 1:numel(results)
    test_records(k).name = results(k).Name;
    test_records(k).passed = results(k).Passed;
    test_records(k).failed = results(k).Failed;
    test_records(k).incomplete = results(k).Incomplete;
    test_records(k).duration_s = results(k).Duration;
    if isfield(results(k).Details,'DiagnosticRecord')
        diagnostics = results(k).Details.DiagnosticRecord;
        messages = arrayfun(@(record) char(record.Report),diagnostics,'UniformOutput',false);
        test_records(k).diagnostics = strjoin(messages,newline);
    end
end
new_test = startsWith({test_records.name},'test_lateral_acceleration/');
legacy_test = ~new_test;
known_stale = strcmp({test_records.name}, ...
    'test_day7_scenario/configuredDay7ScenarioIsLoaded');
report = struct('artifact_kind','lateral_constraint_task1_test_report', ...
    'matlab_executed',true,'new_constrained_search_executed',false, ...
    'new_constrained_trajectory_accepted',false, ...
    'all_tests_passed',all([test_records.passed]), ...
    'new_tests_passed',all([test_records(new_test).passed]), ...
    'legacy_tests_passed',all([test_records(legacy_test).passed]), ...
    'unexpected_legacy_failures',sum([test_records(legacy_test & ~known_stale).failed]), ...
    'known_stale_day7_contract','Expected old priority/goals differ from actual Day 7; test is preserved.', ...
    'tests',test_records,'baseline_diagnostic',struct());
if ~exist(output_directory,'dir'), mkdir(output_directory); end
try
    report.baseline_diagnostic = diagnose_day7_lateral_constraints(source_root, ...
        fullfile(output_directory,'baseline_diagnostic_matlab'));
catch diagnostic_error
    % Keep completed test results available even if the optional legacy-MAT
    % diagnosis cannot run in the current MATLAB installation/environment.
    report.baseline_diagnostic = struct('artifact_kind', ...
        'legacy_saved_trajectory_diagnostic','matlab_executed',true, ...
        'new_constrained_search_executed',false, ...
        'new_constrained_trajectory_accepted',false, ...
        'valid',false,'error_identifier',diagnostic_error.identifier, ...
        'error_message',diagnostic_error.message);
end
file = fullfile(output_directory,'matlab_test_report.json');
fid = fopen(file,'w','n','UTF-8');
if fid < 0, error('LateralAcceleration:OutputOpenFailed','Cannot open %s.',file); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
fprintf('Task 1: %d/%d tests passed; new constrained search was not run.\n', ...
    sum([results.Passed]),numel(results));
end
