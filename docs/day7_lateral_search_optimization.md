# Day 7 横向受限搜索优化

本实验复用 Day 7 横向受限场景与规划器，仅通过独立任务配置改变搜索策略。默认 Day 1–7、Plan C 和横向受限基线配置/输出不变。

## 运行

在 MATLAB R2023a 中将工程设为当前目录后运行：

```matlab
addpath(genpath(fullfile(pwd, 'matlab')));
result = run_day7_lateral_limit_task4( ...
    fullfile(pwd, 'config', 'day7_lateral_optimization_first_feasible.json'));
```

结果写入 `outputs/day7_four_vehicle_lateral_limit_optimization/first_feasible/`。其中包括有效配置快照、MAT/JSON 结果、每车时间统计、采样轨迹、图和动画；四车搜索耗时按该目录 `03_all_four_vehicles/03_all_four_vehicles_time_metrics.csv` 的逐车搜索时间汇总，动画和文件写入不计入四车搜索目标。

## 本次策略

`vehicle_search_overrides` 只接受搜索顺序相关字段：`goal_connection_selection`、`solution_refinement_time_s`、`heuristic_method` 和 `heuristic_weight`。它不能覆盖动力学、车辆几何、安全裕度、碰撞、资源或横向约束配置。没有该字段的旧任务文件保持原运行行为。

当前配置只将 002、004、003 的 Dubins 连接选择从 `lowest_cost` 改为 `first_feasible`。后者在首个连接通过运动学、横向限制、静态碰撞、动态矩形、时空资源与终点驻车检查后返回；这会改变候选代价和可能的路径，但不会跳过安全验收。配置仍使用 4.9 m/s² 横向限制、原车辆与裕度、原搜索预算和原运动学积分。

## 精确重复剖面实验

运行 `config/day7_lateral_optimization_duplicate_profile_pruning.json` 可在独立输出目录测试第二个优化。对每条 Dubins 几何分别保存已完整检查的可行速度相位矩阵；只有后续候选矩阵与已有矩阵由 `isequal` 精确相等时，才跳过重复积分、静态检查和动态/资源检查。不同几何不会合并，近似相等的浮点剖面也不会合并。因为父节点、几何、剖面与环境相同，被跳过的运动学轨迹和代价与先前候选完全一致。

该开关 `goal_connection_duplicate_profile_pruning_enabled` 默认为关闭，旧配置不含字段时保持原行为。实验结果单独记录 `goal_connection_duplicate_profiles_pruned`，并出现在时间指标 CSV 中。运行命令：

```matlab
result = run_day7_lateral_limit_task4( ...
    fullfile(pwd, 'config', 'day7_lateral_optimization_duplicate_profile_pruning.json'));
```

与 `outputs/day7_four_vehicle_lateral_limit_optimization/validated_kernel/` 对比时，使用完整四车阶段的四次实际搜索时间，并检查新增重复剖面计数、每车完整验收、六个车辆对冲突报告。该实验不改速度或加速度上限、横向限值、安全裕度、控制采样与搜索预算。

## 末端状态复用实验

`config/day7_lateral_optimization_endpoint_state_reuse.json` 在独立输出目录重跑完整四车搜索，承接已验证的重复剖面去重设置。`vhybrid_expand_node` 的完整动作积分已经计算动作末端；碰撞采样循环最后一个比例恰为 1，因此本实验直接复用该状态，但仍对同一末端位姿执行静态矩形检查，并将末端状态写入连续动态检测使用的样本缓存。中间采样数量和时刻、平均速度积分、运动学步长、静态/动态矩形碰撞与资源检查都保持原样。

在 MATLAB R2023a 中运行：

```matlab
result = run_day7_lateral_limit_task4( ...
    fullfile(pwd, 'config', 'day7_lateral_optimization_endpoint_state_reuse.json'));
```

结果写入 `outputs/day7_four_vehicle_lateral_limit_optimization/endpoint_state_reuse/`。与 `duplicate_profile_pruning/` 比较四车阶段逐车搜索时间、节点/拒绝计数、路径指标和完整轨迹/六车对验收。此项是结果等价的局部优化，不应改变路径及统计；由于节省量取决于最终采样点调用数，仍以 MATLAB 实测为准。

## 结果判断

验收必须读取实际搜索结果：确认每辆车 `valid`，逐车核对重放、横向与纵向加速度、起始/终端速度和规划时间，并确认六个车辆对的矩形碰撞及资源冲突均为零。超时只表示当前预算内未找到解，不等同于无解。对比基线时使用完整四车阶段的四次搜索耗时；Plan C 的约 19 秒包含已保存 Day 7 搜索耗时和固定路线末段剖面试验，不是四辆车重新搜索。

## 启发式引导实验

`config/day7_lateral_optimization_vehicle003_dubins.json` 将 vehicle_003 的欧氏启发式替换为双向 Dubins 几何下界，并启用精确终点剖面去重。完整四车阶段通过，四车搜索时间约为 `0.963 + 96.853 + 12.833 + 10.772 = 121.42 s`；vehicle_003 从约 111.93 s 降至 10.77 s，扩展节点从 6731 降至 431。路径长度和到达时间允许改变，硬约束、碰撞、资源和终端停车验收仍执行。

`config/day7_lateral_optimization_vehicle002_weighted.json` 在上述设置基础上对 vehicle_002 使用双向 Dubins 启发式权重 8，并保持 vehicle_003 的权重 3。该配置是当前推荐的独立受限搜索配置，完整四车阶段实际耗时约 `0.867 + 6.094 + 12.530 + 8.331 = 27.82 s`，低于 60 s 目标，结果 `valid=true`：

| 车辆 | 搜索耗时 (s) | 扩展/生成节点 | 路径长度 (m) | 到达时间 (s) | 最高速度 (m/s) | 峰值横向加速度 (m/s²) |
|---|---:|---:|---:|---:|---:|---:|
| 001 | 0.867 | 11 / 130 | 30.410 | 7.287 | 7.757 | 4.900 |
| 002 | 6.094 | 93 / 627 | 26.410 | 9.298 | 6.000 | 4.394 |
| 004 | 12.530 | 231 / 2475 | 23.132 | 6.059 | 5.191 | 4.900 |
| 003 | 8.331 | 308 / 1985 | 34.704 | 9.223 | 5.191 | 4.900 |

相对于当前最快的等价 `endpoint_state_reuse` 实验约 230.10 s，该配置约快 8.3 倍；相对于同一轮横向受限基线约 390.98 s，约快 14.1 倍。路线、速度和到达时间发生改变，这是加权启发式搜索的取舍，不宣称全局最优；没有降低横向限值、车辆安全裕度、碰撞检查精度或运动学步长。

曾测试 `config/day7_lateral_optimization_weighted_guidance.json`，虽然 002/004 很快，但 003 在 480 s 搜索预算内未返回合法路径（错误码 6，约 20,925 个扩展节点）；该配置保留为失败诊断，不作为推荐结果。

运行推荐配置：

```matlab
addpath(genpath(fullfile(pwd, 'matlab')));
result = run_day7_lateral_limit_task4( ...
    fullfile(pwd, 'config', 'day7_lateral_optimization_vehicle002_weighted.json'));
```

推荐结果索引为 `outputs/day7_four_vehicle_lateral_limit_optimization/vehicle002_weighted/output_index.json`。该目录包含实际生效配置快照、动画、轨迹采样 CSV、时间指标 CSV、MAT、JSON、XY 图、速度图和横向加速度图。`day7_task4_final_time_metrics.csv` 是四车搜索耗时和拒绝统计的直接来源。

## 运行器诊断修复

`run_day7_lateral_limit_task4` 增加稳定的失败原因转换函数。搜索错误码、验收失败原因和规划器诊断现在分开记录；超时仍报告为搜索预算耗尽，不被解释为数学无解。
