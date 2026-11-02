# Day 7 方案 C：固定路线扩展速度剖面

## 目标与结论

方案 C 复用既有四车 V-Hybrid A* 路线，只重规划 vehicle_002 从 t=4 s 低速节点到终点的速度剖面。候选剖面支持巡航、减速停车、等待、重新加速和终端减速。结果显示：

- 到达时间：16.418728 s 降至 10.097909 s，缩短 38.50%（6.32 s）。
- 最长连续低速段：12.085395 s 降至 1.416667 s；低速总时间 12.418729 s 降至 1.916668 s。
- 实际控制剖面：巡航 3 m/s，在弧长 7.5% 处停车等待 1 s，随后重新加速。
- 路线保持：弧长逐点重放最大位置偏差 2.56e-14 m，最大航向偏差 1.24e-14 rad。
- 安全验收：四车起步、终端驻车、运动学重放、6/6 车对矩形碰撞与时空资源冲突全部通过。

## 实现方式

vhybrid_goal_connection 在原单相位连接之后增加独立的多相位候选循环，配置开关为 goal_connection_multi_phase_profile，默认关闭。候选由三个集中配置向量组合生成：巡航速度、停车弧长比例、等待时间。剖面解析构造并做速度连续性和总弧长校验；等待段生成真实的零速时间节点。每个候选复用 Day 6 的连续段矩形碰撞检查、时空资源块查询和目标驻车占用检查；代价剪枝与普通连接一致。

多相位候选只有真正通过全部安全检查且总代价更低时才会替换原连接。选中的剖面会记录巡航速度、停车比例、等待时间、候选数、碰撞剪枝数、代价剪枝数和等待时长。summon_vhybrid_astar 同步累积这些统计。

## 使用方法

```matlab
result = run_day7_plan_c_experiment(); % 缺省读取 config/day7_plan_c_experiment.json
```

实验配置 config/day7_plan_c_experiment.json 指定基线四车轨迹文件、ego 车辆与连接起始时间、低速阈值、路线重放容差和方案 C 候选参数。实验输出独立保存在 outputs/day7_plan_c/，不覆盖 outputs/day7_four_vehicle/。

## 输出与查看

| 文件 | 内容 |
| --- | --- |
| outputs/day7_plan_c/day7_vehicle_timing.csv | 每车到达时间、规划耗时、起步时间 |
| outputs/day7_plan_c/day7_trajectory_samples.csv | 四车完整时空采样表（x/y/θ/v/t） |
| outputs/day7_plan_c/day7_four_vehicle_animation.gif | Day 6 同款四车动画（含冲突安全圈） |
| outputs/day7_plan_c/day7_plan_c_comparison.csv 与 .png | 基线与方案 C 的低速/到达指标对照 |
| outputs/day7_plan_c/day7_plan_c_result.json | 完整验收报告、剖面统计、车对冲突明细 |
| outputs/day7_plan_c/day7_plan_c_trajectories.mat | 更新后的四车路径结构体 |
| outputs/day7_plan_c/day7_plan_c_baseline.mat | 原始基线与配置快照，保证实验可复现 |

## 测试

tests/test_vhybrid_multi_phase.m 使用真实 Day 7 基线和 vehicle_001 高优先级资源验证：缺省关闭行为与移除开关完全一致、真实冲突场景选择停车等待、代价剪枝结果与全枚举一致、等待向量缺省或为空时退化为零等待、无冲突时保持更优单相位剖面，以及门控、速度、比例、等待向量的非法输入显式报错。

运行 runtests 覆盖 test_vhybrid_multi_phase、test_vhybrid_speed_phases、test_vhybrid_goal_radius、test_dynamic_collision、test_reverse_initial_gear：164 项全部通过。

## 边界说明

方案 C 不改变其他三车路线和时间，也不调整优先级、搜索步长或资源占用语义。若基线重新生成，需重跑本实验；脚本会重新加载基线并独立复核全部验收。整车搜索耗时统计仍保留基线值，剖面追加耗时单独记录，不混入原 V-Hybrid A* 计时。
