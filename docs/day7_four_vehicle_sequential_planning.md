# Day 7：四车顺序规划、路径质量与输出

## 实现范围

Day 7 在独立场景中复用 Day 6 的 `summon_vhybrid_astar`、车辆运动学、静态矩形判定、X-Y-T 资源块和动态车辆碰撞实现。Day 7 的配置、输出目录独立；共享搜索器与目标连接器增加可选路径质量设置，缺省设置保留原有首次可行解行为。Day 6 的场景、配置和输出基线不由 Day 7 入口覆盖；共享代码的兼容性应由回归测试确认。

入口为 `matlab/single_vehicle/run_day7_sequential_planning_demo.m`。在工程根目录运行：

```matlab
addpath(fullfile(pwd,'matlab','single_vehicle'));
result = run_day7_sequential_planning_demo(true);
```

配置分别位于 `config/vehicle_config_day7.json`、`config/map_config_day7.json`、`config/planner_config_day7.json` 和 `config/scenario_day7_four_vehicle.json`。

## 场景与出发语义

地图为 `[0,30] × [0,30] m`，资源块采用半开区间约定，当前空间和时间分辨率均为 `0.5`，分别以 m 和 s 为单位。车辆尺寸、轴距、转角限制及前后侧安全裕度沿用 F515 配置。位姿坐标为后轴中心，航向角单位为 rad。

| 车辆 | priority | 起点 `[x,y,yaw]` | 终点 `[x,y,yaw]` |
|---|---:|---|---|
| vehicle_001 | 1 | `[4,25,-1.57]` | `[25,5,-1.57]` |
| vehicle_002 | 2 | `[8,26,0]` | `[20,6,-1.57]` |
| vehicle_003 | 3 | `[27,25,3.14]` | `[6,4,3.14]` |
| vehicle_004 | 4 | `[23,20,-1.57]` | `[28,8,-1.57]` |

按用户授权的“无解时可调整起终点”条款，vehicle_004 的目标从最初的 `[10,8,-1.57]` 调整为 `[28,8,-1.57]`。原目标试验在 120 秒和 240 秒搜索预算下均未找到路径，搜索主要受车辆 001/002 的中央时空占用限制；向东侧移动目标后，同一 Day 6 搜索器在保留立即起步、完整碰撞检查和终点驻车占用的情况下找到有效路径。目标仍通过地图边界和车身矩形检查。

搜索前检查八个起终点安全车身矩形的地图边界、静态碰撞，检查四车初始矩形之间是否重叠，并验证时空配置。检查结果记录在 `pose_validation`。任一检查失败时不开始正常顺序搜索。

四辆车均从 `t=0,v=0` 开始，首段立即运动，终端速度为零。车辆按 priority 排序；每完成一条真实搜索轨迹，就将其插值车身矩形、连续段扫掠以及终点驻车至资源图时间上限的占用写入共享地图，再规划下一辆车。没有手工延迟、预设路径或按车辆自动调整速度。

`initial_gear_mode='auto'` 允许正、倒挡根节点在同一个搜索中从 `t=0` 参与竞争；起步时不人为增加换挡等待。运行中的换挡仍须停车并满足配置的换挡时间。状态 `v` 为非负速度幅值，带方向速度为 `v*gear`。

## 末端绕圈的原因与改进边界

原目标连接器仅使用最小转弯半径生成 Dubins 曲线，按几何长度尝试曲线，并依次尝试巡航速度，找到完整可行连接就返回。原搜索主循环也在得到第一条通过完整验收的路径后返回。因此，即使节点已接近终点，当其接近方向不理想时，首次可行连接仍可能包含很长的终端转弯。最小转弯半径、精确终点航向和零终速又会限制可选连接，不能把所有必要圆弧都视作错误。

仅调大航向代价不保证消除绕圈：如果搜索仍在首条可行路径处停止，尚未访问的更好接近方向没有机会被比较。本次增加的可选机制分别处理同一父节点的连接质量，以及不同父节点之间的完整路径质量：

| `vhybrid` 配置字段 | 缺省值 | 含义 |
|---|---|---|
| `goal_connection_selection` | `'first_feasible'` | `'lowest_cost'` 在同一父节点枚举的可行连接中比较完整累计 `g` |
| `goal_connection_radius_scales` | `[1]` | 各值必须有限且不小于 1；使用 `Rmin*scale` 生成几何候选 |
| `goal_connection_cost_pruning_enabled` | `true` | 仅用于 `lowest_cost`；满足实际增量代价非负条件时，提前停止已不可能改进当前连接代价的积分前缀 |
| `solution_refinement_time_s` | `0` | 大于零时，在首条完整有效解之后继续原 Open/Closed/搜索树，保留累计 `g` 更小的有效解 |

扩大半径意味着更小转角，仍须满足原最大转角约束。多半径只增加有限候选，不能保证每个场景都更短。`lowest_cost` 只比较同一父节点已枚举的候选；继续搜索改进解才可能找到接近方向更好的其他父节点。两者均不代表全局最优。

实际代价由距离（含倒车倍率）、速度变化、参考速度偏差、绝对航向变化、时间及换挡惩罚累计得到，权重均来自配置。`g` 是比较最终路径的目标值；`h` 只影响搜索顺序。入口将 `day6.search_heuristic_weight` 和 `day6.heuristic_method` 写入 V-Hybrid 的有效启发设置，应以这两项解释演示的搜索顺序。增大启发权重可能降低搜索耗时，也可能降低路径质量。

改进预算从 `first_solution_time_s` 开始计时，找到更好解不会重置预算。最大总搜索时间、最大节点数仍然生效。预算在主循环边界检查，不强制中断正在进行的目标连接或完整验收，因此可能略超出配置的改进时间。到达改进预算、搜索时间、节点数或 Open 耗尽时，如果已有完整有效解则返回保留解，否则按正常失败流程报告。`refinement_stop_reason` 区分 `disabled`、`refinement_time_limit`、`search_time_limit`、`node_limit` 和 `open_exhausted` 等情况。

所有候选仍须通过静态碰撞、动态车身矩形、资源图、终点驻留、终端状态和运动学重放检查。提高路径质量不放宽安全条件，也不删除终点航向和停车要求。

## 低优先级车辆耗时的来源

低优先级车辆面对更多已规划车辆的移动占用和终点长期驻车占用。状态包含 `x/y/yaw/v/t`，并区分挡位；控制组合、时间离散和冲突拒绝共同扩大搜索量。目标连接又反复进行曲线生成、速度积分、矩形检查和资源查询，可能在较少扩展节点时仍消耗较长时间。多半径、最低代价连接和首解后改进也会增加计算。

共享连接器会预先检查整段圆弧的车身角点边界极值；已确定几何曲线静态碰撞时，跳过同一曲线后续速度尝试。在满足条件时，累计实际代价也可用于连接内部剪枝。这些检查不能代替动态碰撞或最终验收。

评估耗时应同时查看首解时间、总规划时间、扩展/生成节点数、动态和资源拒绝统计。`dynamic_pruned` 与 `resource_pruned` 可能覆盖同一候选，不能相加当作互斥总数。单纯提高最大时间和节点数只扩大计算预算，不保证更短路径或更快收敛。

## 故障降级接口

默认运行不注入故障。真实外部故障通过 `options.fault_event` 提供：包含故障原因、故障车辆、受影响车辆，以及各受影响车辆从场景起点开始的真实可重放控制前缀和安全停车状态 `[x,y,yaw,0,t]`。

入口把受影响车辆移至队列末尾，保留其相对顺序；先预留停车前缀占用，再规划未受影响车辆和停车车辆的剩余路径，最后合并前后缀并重新验收。缺少有效前缀或停车状态时返回 `invalid_fault_event`。普通无解、超时和 Open 耗尽不自动作为故障事件，也不触发无限重排。这里不包含在线故障检测、控制器或全局优先级优化。

## 动画、时间和路径表

所有 Day 7 文件位于 `outputs/day7_four_vehicle/`：

| 文件 | 内容 |
|---|---|
| `day7_four_vehicle_animation.gif` | 与 Day 6 风格一致的四车同步运动动画 |
| `day7_animation_preview.png` | 动画预览帧 |
| `day7_vehicle_timing.csv` | 每车状态、耗时、长度、转向量、搜索统计和终端结果，Excel 可直接打开 |
| `day7_trajectory_samples.csv` | 每车逐采样点完整路径表，Excel 可筛选和绘图 |
| `day7_four_vehicle_report.json` | 配置检查、失败原因、冲突车辆/时间/位置、搜索统计、车对验收与计时 |
| `day7_four_vehicle_trajectories.mat` | `paths`、`result`、`timings`，便于 MATLAB 后续分析 |
| `day7_four_vehicle_trajectories.png` | 地图、四车路径、起终点与车身矩形 |
| `day7_four_vehicle_speed_time.png` | 四车带方向速度随时间变化 |
| `day7_validation.json` | 完整验收入口写出的测试与真实搜索汇总 |

逐车摘要表每次运行均输出。失败及尚未规划车辆的未知到达状态、长度等为 `NaN`，并保留 `status`、`error_code` 和 `joint_validation_passed`。GIF、预览和完整采样 CSV 只在四条路径均有效且联合验收通过后生成；失败重跑时清除旧成功动画和完整采样 CSV，避免混用结果。

摘要指标的含义：

- `planning_time_s`：单次搜索调用的实际计算耗时，包含调用内的完整路径验收。
- `first_solution_time_s`：从本次搜索开始到首次完整有效解的计算耗时；与总耗时比较可区分找解和继续改进的开销。
- `first_solution_cost`、`final_solution_cost`：首解和返回解的加权累计代价，不以 m 或 s 为单位；`solution_improvements` 统计更优有效解次数。
- `refinement_stop_reason`：首解后继续搜索的停止原因。
- `arrival_time_s`：路径末端的仿真时间；`path_duration_s` 为末端减起始仿真时间，均不是规划计算耗时。
- `path_length_m`：优先累计各段 `abs(travelled_distance)`，缺失时用采样折线长度。
- `total_abs_yaw_rad`：累计相邻采样点归一化至 `[-pi,pi)` 后的绝对航向变化，用于观察绕圈和转向量。
- `expanded_nodes`、`generated_nodes`、`dynamic_pruned`、`resource_pruned`：搜索规模和拒绝统计。

完整路径 CSV 按车辆和采样时间列出 `sample_vehicle_id, sample_priority, sample_index, time_s, x_m, y_m, yaw_rad, speed_mps, signed_speed_mps, acceleration_mps2, steering_angle_rad, gear`。MAT 文件不写入完整扩展节点云以控制大小；函数返回值保留搜索节点。读取示例：

```matlab
saved = load('outputs/day7_four_vehicle/day7_four_vehicle_trajectories.mat');
vehicle1_path = saved.paths{1};
saved.result.timings
```

`planning_s` 为各车搜索调用耗时之和。整体运行还包含配置检查、联合验收、绘图、动画及文件写入。`file_output_s` 计入主要轨迹数据写入，不重复计入随后元数据和 JSON 写入。

动画沿用 `day6.animation_step_s` 和 `day6.animation_frame_rate`。例如每帧推进 `0.2 s`、每秒播放 10 帧时为约 2 倍仿真速度；动画播放时长不能用来推断搜索计算耗时。

## 测试与验收入口

配置、故障接口、共享核心、目标连接与有界改进测试：

```matlab
results = runtests({ ...
    'tests/test_day7_scenario.m', ...
    'tests/test_day7_fault_replan_order.m', ...
    'tests/test_vhybrid_core.m', ...
    'tests/test_vhybrid_goal_radius.m', ...
    'tests/test_vhybrid_refinement.m'});
```

目标连接测试覆盖缺省和单位半径的正/倒车兼容、多半径运动学重放、最低代价模式和剪枝等价性、非法配置。改进测试覆盖缺省与零预算兼容、保留有效且不劣的解、极小预算、搜索上限、Open 耗尽、冲突和非法预算。

完整四车验收会重新执行实际搜索：

```matlab
addpath(fullfile(pwd,'tests'));
summary = validate_day7_sequential(true);
```

该入口执行 Day 7 场景与故障测试及真实四车验收；新增共享核心/目标连接/改进套件仍应按上方列表单独运行。验收应检查四条路径、运动学重放、起始 `t=0,v=0`、首段立即运动、终端零速，以及全部六组车辆对的连续矩形碰撞和独立时空资源冲突。失败时保留明确诊断，不能以动画生成或单车 `valid` 代替联合通过。

## 最终统一配置与验收结果

以下项目需由同一套最终配置、同一次四车运行和对应回归测试填入。旧版本的运行时间、通过数量和最小间距不作为本轮结果。

- 最终配置：`vhybrid.goal_connection_distance_m=35 m`、`goal_connection_max_turn_rad=pi`、连接器采用 `lowest_cost` 且启用代价剪枝；vehicle_003 使用 Euclidean 搜索引导和 5 s 首解改进预算，vehicle_004 使用启发权重 1、倒车倍率 0.5 和 2 s 改进预算；总搜索上限 240 s、200,000 节点。其余车辆使用 Day 6 的启发权重和方法。Day 6 的碰撞、运动学、立即起步和终点驻车判据未放宽。
- MATLAB 测试：Day 7 配置/故障/核心/目标连接/细化/速度剖面/启发函数套件共 **143 passed, 0 failed, 0 incomplete**。`validate_day7_sequential(true)` 另行运行的 Day 7 场景及故障排序测试为 **9/9**，并完成实际四车规划验收。
- 最终四车入口结果：`result=passed`；搜索耗时合计 **22.597 s**，整次输出 **74.837 s**（其中同步 GIF 编码 47.882 s）。四车全部有效，从 `t=0,v=0` 立即起步，运动学重放通过，终端速度为零。
- 每车结果：vehicle_001 到达 8.544 s、30.222 m、搜索 0.148 s；vehicle_002 到达 17.103 s、24.529 m、搜索 0.784 s；vehicle_003 到达 57.635 s、31.818 m、搜索 7.170 s；vehicle_004 到达 14.460 s、13.355 m、搜索 14.495 s。vehicle_003 的 5 s 改进预算内未找到更低代价解；vehicle_004 在 2 s 改进阶段找到 1 条更低代价解。有限控制采样和搜索预算不保证全局最优。
- 六组车对均通过连续矩形碰撞及独立资源块冲突检查，资源重叠块均为 0。连续最小矩形距离：001/002 2.066 m、001/003 6.927 m、001/004 1.147 m、002/003 2.280 m、002/004 5.173 m、003/004 3.070 m。最小间距来自 001/004，但通过现有 Day 6 安全矩形判据。
- 本次动画、计时 CSV、逐点路径 CSV、MAT 和报告均位于 `outputs/day7_four_vehicle/`，全部由同一次通过验收的入口运行生成。

当前实现不包含全局联合优化、轨迹平滑和控制器；有限控制采样、离散状态及有界搜索不提供全局最优保证。
