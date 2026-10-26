function report = validate_day7_sequential(save_outputs)
%VALIDATE_DAY7_SEQUENTIAL 运行Day7故障排序测试和真实四车场景验收。
% 输出报告仅写入outputs/day7_four_vehicle，不覆盖Day6场景及结果。
if nargin<1,save_outputs=true;end
project=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project,'matlab','single_vehicle'));
test=runtests({fullfile(project,'tests','test_day7_fault_replan_order.m'), ...
    fullfile(project,'tests','test_day7_scenario.m')});
report=struct('matlab_version',version,'checked_at',char(datetime('now')), ...
    'test_count',numel(test),'passed',nnz([test.Passed]), ...
    'failed',nnz([test.Failed]),'incomplete',nnz([test.Incomplete]), ...
    'test_names',{{test.Name}},'unit_tests_passed',false, ...
    'planning_complete',false,'valid',false,'acceptance_failure','');
if any([test.Failed]) || any([test.Incomplete])
    error('Day7:UnitTestsFailed','Day7优先级单元测试未全部通过。');
end
report.unit_tests_passed=true;
demo=run_day7_sequential_planning_demo(save_outputs);
report.demo=rmfield(demo,'paths');
report.valid=demo.valid;
report.planning_complete=all([demo.per_vehicle.valid]);
if ~demo.valid
    report.acceptance_failure=demo.reason;
end
if save_outputs
    output=fullfile(project,'outputs','day7_four_vehicle','day7_validation.json');
    if ~exist(fileparts(output),'dir'),mkdir(fileparts(output));end
    fid=fopen(output,'w','n','UTF-8');
    assert(fid>=0,'Day7:OutputFailure','无法保存Day7验收报告。');
    cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
end
end
