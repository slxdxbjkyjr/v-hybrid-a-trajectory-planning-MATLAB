# Day 7 横向加速度约束：任务一

本阶段实现共享动力学约束和逐段轨迹验收，不修改 V-Hybrid A* 搜索策略、不执行路径重规划，也不实现速度 QP 或路径优化。现有平均速度积分 `ds = 0.5*(v0+v1)*dt` 保持不变。

## 约束和接口

新集中配置字段为 `dynamics.lateral_acceleration_constraint_enabled`、`dynamics.max_lateral_acceleration_mps2` 和 `dynamics.lateral_acceleration_tolerance_mps2`。Day 1–7 旧配置没有启用字段时按关闭处理；启用值必须是 logical 标量，启用后限值必须是有限正数。数值容差默认为 `1e-9 m/s^2`，允许范围为 `[0,1e-6]`。

`summon_lateral_acceleration` 是统一计算内核。对恒转角、恒加速度段计算
`max(abs(v_start),abs(v_end))^2 * abs(tan(delta)/L)`，并分别检查入口和出口。`summon_vehicle_dynamic` 保留原十参数接口，可选第十一参数传入 dynamics；`summon_vehicle_dynamic_gear` 保持原档位接口并复用此约束，覆盖前进和倒车。未配置启用字段的调用保持旧行为。

`vhybrid_validate_lateral_acceleration` 从每个段的控制量重新计算曲率与峰值，不读取缓存曲率或已有有效标志；同时比较轨迹记录的末速度和由恒加速度重算的末速度。轨迹控制 `k` 作用于样本 `k-1 → k`，因此新转角以入口速度验收。`vhybrid_validate_trajectory` 将逐段结果放在 `report.lateral_acceleration`，并将超限或非法段纳入总约束失败数。非法段报告包含样本序号、时间、位置、速度、曲率、峰值和原因。

## 隔离配置和现有基线

新增 `config/planner_config_day7_lateral.json` 和 `config/scenario_day7_lateral.json`，引用原 Day 7 地图和车辆配置，只启用 4.9 m/s² 限值、数值容差、隔离输出目录，并将 `require_immediate_start` 设为 false。车辆优先级、位姿、轴距、转角和安全裕度与实际 Day 7 场景一致；车辆 004 终点为 `[6,10,-3.14]`。原始 Day 1–7 配置、输出和测试未修改。

当前 Day 7 demo runner 仍硬编码使用原配置及 `outputs/day7_four_vehicle`。本阶段没有将隔离配置接入 runner，也没有启动新搜索；后续接搜索时需要提供可指定配置和独立输出目录的入口。

## 测试入口和诊断

MATLAB 可用时，从工程根目录执行：

```matlab
addpath(fullfile(pwd,'matlab','single_vehicle'));
report = run_lateral_constraint_task1(pwd, ...
    fullfile(pwd,'outputs','day7_lateral_constraint','task1'));
```

该入口包括新增横向约束测试、旧动力学/倒车回归和既有 Day 7 场景测试；新增测试另检查受限配置与原配置差异、真实车辆顺序/目标、零速初态和合法等待。原 `test_day7_scenario/configuredDay7ScenarioIsLoaded` 的优先级和 vehicle 004 目标仍断言旧值，按要求保留，报告中会单独注明。

`diagnose_day7_lateral_constraints` 只读原 Day 7 MAT 和原配置，把诊断 JSON 写入独立输出目录，并在诊断前后比较源文件 SHA-256。MATLAB 阻塞时还可用 `tools/diagnose_day7_lateral_csv.ps1` 对原 CSV 做独立公式诊断。两者均标记为旧轨迹诊断，不是新受限搜索结果或新轨迹验收。

本次 PowerShell CSV 只读诊断覆盖原轨迹 1264 段，发现 113 段超过 4.9 m/s²：vehicle 001 为 41 段（峰值 12.6039），002 为 0 段（3.6818），004 为 48 段（11.0917），003 为 24 段（6.5455）。MATLAB 重新读取保存的 MAT 并独立计算，得到相同的各车违规段数和峰值。两份结果位于 `outputs/day7_lateral_constraint/task1/baseline_diagnostic_csv/` 和 `outputs/day7_lateral_constraint/task1/runner_verification/baseline_diagnostic_matlab/`；原始 Day 7 输出没有被覆盖。它们只诊断保存的旧轨迹，不代表新搜索或受限验收。

MATLAB R2023a 已执行 `tests/execute_lateral_constraint_task1.m`：115 项中 114 项通过。70 项新增约束/配置测试、8 项旧车辆动力学测试、19 项倒车测试和 14 项初始档位测试全部通过；唯一失败是上文记录的旧 `test_day7_scenario/configuredDay7ScenarioIsLoaded` 断言。测试报告位于 `outputs/day7_lateral_constraint/task1/runner_verification/matlab_test_report.json`，其中新增测试全通过、非预期旧回归失败为 0。独立运行 `test_day7_scenario.m` 得到 3 通过、1 失败，失败原因相同。

MATLAB Code Analyzer 已检查共享动力学、配置校验、逐段验收和测试入口的主要 MATLAB 函数；除诊断函数和 runner 上已消失的历史信息提示外，没有活动代码问题。原工程 165 个基线文件中，除按计划更新的三个共享函数外，其余 162 个哈希均未改变；MATLAB 诊断前后读取的原 MAT 和三份配置哈希也一致。新搜索仍未运行，本阶段没有新的受限轨迹可验收。
