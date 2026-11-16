# 四车统一策略与多场景从零规划

## 目标与边界

四车共用一套车辆参数、动力学、控制采样、启发式、终点连接和搜索预算。场景只提供起终位置、航向和规划顺序；车辆编号只用于标识、占用归属和输出。当前地图为原30×30 m地图，横向限制4.9 m/s²、纵向限制[-6,4] m/s²、最高速度10 m/s和车辆0.2 m安全裕度保持不变。继续使用既有平均速度积分，搜索步长及碰撞精度不变。

每次调用新建Open、Closed、搜索节点与时空地图。按场景顺序从t=0、v=0规划，允许搜索等待；把本次新生成的前车真实轨迹和终点驻车占用加入地图后再搜索下一车。不读取历史MAT/CSV轨迹作初值，不执行Plan C固定路线实验。重复剖面只在同次连接、同条几何内精确去重，末端状态仅复用本次动作的积分结果。

## 原实现的问题与处理

1. 原任务四包装函数固定004目标及车辆顺序。旧入口继续保留，新入口显式启用`uniform_shared`，以通用输入校验取代固定坐标检查。
2. 原任务配置和基础配置都存在逐车覆盖。新公共配置不继承这些覆盖；新入口遇到逐车覆盖或历史初值字段直接报错。公共启发式只从`vhybrid`读取，不再被`day6`同名设置覆盖。
3. 原场景priority与任务planning_order冲突。新场景planning_order是唯一顺序，priority仅作为派生输出。
4. 按用户授权，公共配置启用`dynamic_braking_dominance_pruning_enabled`：同一父节点、挡位和转向按加速度升序检查，减速段中间发生动态矩形或资源冲突后，跳过该转向尚未检查的更快动作；其他转向继续扩展。静态减速中间碰撞沿用旧规则。仅末端碰撞仍分别检查后续动作。该规则是可能漏掉可行更快动作的启发剪枝，不是可行性支配证明；所有返回动作仍完整检查。缺少字段时保留旧规则，显式false关闭动态部分。
5. 增加终点安全矩形相互重叠预检，防止在终点持续驻车不兼容时耗尽搜索预算。输入不合法、预算耗尽和冲突证据分别记录。
6. 首个运动段开始时刻由段入口时间判定，修正原来用第一个正速度采样时刻误判立即起步的诊断。允许等待设置不变。

## 配置和运行

在MATLAB R2023a中将工程设为当前目录：

```matlab
addpath(genpath(fullfile(pwd,'matlab')));
result = run_four_vehicle_lateral_planning();
```

任务配置：`config/four_vehicle_lateral_uniform.json`。
公共参数：`config/planner_config_four_vehicle_uniform.json`。
当前场景：`config/scenario_four_vehicle_uniform_current.json`。

冻结的公共策略为双向Dubins引导、权重8、`first_feasible`、首解后改进时间0、精确重复剖面去重开启、动态减速中间碰撞剪枝开启。方案C多相位连接显式关闭，普通连接本身已有曲率速度包络；此开关不影响旧配置和旧方案C。每车预算一致：200000节点、60秒；运动时空范围仍为0至60秒。加权启发和首个可行连接优先降低计算量，不保证最短路线或最早到达。

修改场景文件的`vehicles(k).start_pose`、`summon_goal`和`planning_order`即可输入新场景。位姿坐标参考后轴中心，距离m、航向rad；有限航向统一归一化到[-pi,pi)。顺序必须恰好包含四个唯一编号，不需要使用vehicle_001等固定命名。任务结构仅接受场景/配置引用和输出位置，不接受逐车参数。`day7_uniform_configuration`提供不运行搜索的公共输入校验。

```matlab
task = jsondecode(fileread('config/four_vehicle_lateral_uniform.json'));
task.scenario_config = 'scenarios_uniform/rotate90.json';
result = run_four_vehicle_lateral_planning(task);
```

默认每次在`outputs/four_vehicle_lateral_uniform/`下新建带时间戳和唯一后缀的目录。包含有效配置快照、每车实际公共配置、MAT/JSON、轨迹CSV、指标CSV、XY/速度/横向加速度图和动画，入口返回`result.output_files.output_index`。旧Day7、Plan C及受限基线目录不会被覆盖。

## 测量与Profiler

```matlab
% 无文件/绘图输出的完整搜索，仍执行运动学和安全验收
result = run_four_vehicle_lateral_planning(task,false);
% 可选第三参数：每完成一辆写入进度JSON，不导出图也可使用
result = run_four_vehicle_lateral_planning(task,false,'progress.json');
% 一轮预热，随后三轮独立搜索；每轮都不复用上轮路径
warmup = benchmark_four_vehicle_lateral(task,1,false);
measured = benchmark_four_vehicle_lateral(task,3,false);
% 独立剖析，不把Profiler时间与正常时间混算
profiled = benchmark_four_vehicle_lateral(task,1,true);
```

四车搜索时间包含四次搜索调用内部验收及逐车独立重放，不含跨车最终审计、占用写入、配置加载、绘图或导出。完整6车对审计仍是有效结果的必要条件，其开销不得隐藏为搜索优化。Profiler保存函数总时间、自身时间、调用数和原始MAT；总时间含子调用，不可直接相加。

原27.82 s结果使用逐车权重3/8/1/3和不同连接策略，只作历史性能参考，不作为本版本的四车同配置验收。新统一策略已完成预热和三次独立测量，详细证据见`four_vehicle_lateral_uniform_results.md`。统计成功率时保留失败、超时和输入不合法场景，不将失败的短耗时计为成功加速。

## 已确认的6场景集

`config/four_vehicle_uniform_suite.json`记录用户确认的当前场景、绕(15,15)旋转90/180/270度、沿x=15镜像、全部起终航向加0.2 rad六个输入。配置文件位于`config/scenarios_uniform/`，所有场景使用同一份冻结公共策略，不按场景调参。该集合是首批确定性验收，不代表所有任意场景或随机分布的成功率。

```matlab
suite = run_four_vehicle_uniform_suite();
% 不生成各场景动画/图，仍保存套件指标和真实结果
suite = run_four_vehicle_uniform_suite(false);
```

每个场景先执行安全矩形输入检查，成功轨迹执行整段横向、纵向、模型重放、起终状态和六个车对的连续矩形及资源检查。固定顺序的优先级规划不保证所有可行四车场景都能成功；不擅自换顺序、降低限制或手工延迟，预算耗尽只表明本次没有找到合法解。

## 回归测试

```matlab
r = runtests({'tests/test_four_vehicle_uniform.m', ...
    'tests/test_dynamic_collision.m','tests/test_lateral_acceleration.m', ...
    'tests/test_lateral_search_integration.m', ...
    'tests/test_vhybrid_lateral_connection_profile.m', ...
    'tests/test_vhybrid_heuristic.m','tests/test_reverse_search.m'});
assert(all([r.Passed]) && ~any([r.Incomplete]));
```

测试包括统一策略、拒绝隐藏覆盖与历史初值、任意编号与航向、起终点非法输入、终点驻车重叠、编号互换后的实际新搜索一致性，以及移动障碍下减速冲突但加速合法的反例。还覆盖启用剪枝时跳过更快动作、不影响其他转向、倒车和仅末端碰撞时继续独立检查。`braking_collision_pruned`统计触发次数，`faster_controls_skipped`统计实际跳过的控制数，均汇总到搜索统计及benchmark CSV。旧动力学、横向约束、资源查询和倒车测试继续执行。
