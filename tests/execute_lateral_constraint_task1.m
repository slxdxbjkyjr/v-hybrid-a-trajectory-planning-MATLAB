%EXECUTE_LATERAL_CONSTRAINT_TASK1 Run Task 1 tests against the installed project.
% This script runs tests from the isolated staging copy and only uses source_root
% for the read-only saved Day 7 trajectory diagnosis.
stage_root = fileparts(fileparts(mfilename('fullpath')));
source_root = ['F:\study\postgraduate\联培\路径规划论文\代码\辅助\day 4\' ...
    'MATLAB-9.11-45973bf338437f8ae7b9fe154fc587f714a4c0df'];
addpath(fullfile(stage_root,'tests'));
report = run_lateral_constraint_task1(source_root,fullfile(stage_root, ...
    'outputs','day7_lateral_constraint','task1','runner_verification'));
fprintf('Task 1: %d/%d tests passed; known stale Day 7 expectation remains separate.\n', ...
    sum([report.tests.passed]),numel(report.tests));
disp(report.baseline_diagnostic);
