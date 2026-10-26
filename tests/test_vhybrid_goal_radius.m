classdef test_vhybrid_goal_radius < matlab.unittest.TestCase
    %TEST_VHYBRID_GOAL_RADIUS 验证多半径/代价择优连接及原行为兼容。
    % 仅调用公开接口；在独立无障碍地图上逐边重放真实车辆运动学。

    properties
        VehicleConfig
        PlannerConfig
        Map
    end

    properties (TestParameter)
        gear = struct('forward',1,'reverse',-1)
        invalidScales = struct('empty',[],'zero',0,'negative',-1, ...
            'belowMinimum',0.9,'nan',[1,NaN],'infinite',[1,Inf], ...
            'matrix',ones(2),'text','1','logical',true, ...
            'complex',1+1i,'radiusOverflow',realmax)
        invalidSelection = struct('empty','','unknown','shortest', ...
            'uppercase','LOWEST_COST','numeric',1,'logical',true, ...
            'charMatrix',['fi';'rs'], ...
            'stringArray',["first_feasible","lowest_cost"], ...
            'cell',{{'first_feasible'}})
        invalidPruning = struct('empty',[],'numeric',1,'text','true', ...
            'vector',[true,false])
        invalidMaxTurn = struct('empty',[],'zero',0,'negative',-1, ...
            'nan',NaN,'negativeInfinity',-Inf,'complex',1+1i, ...
            'vector',[1,2],'matrix',ones(2),'text','Inf','logical',true)
        unsafeCost = struct( ...
            'negativeDistance',struct('field','weight_distance','value',-1), ...
            'negativeSpeedChange',struct('field','weight_speed_change','value',-1), ...
            'negativeReferenceSpeed',struct('field','weight_reference_speed','value',-1), ...
            'negativeHeadingChange',struct('field','weight_heading_change','value',-1), ...
            'negativeTime',struct('field','weight_time','value',-1), ...
            'nanDistance',struct('field','weight_distance','value',NaN), ...
            'infiniteSpeedChange',struct('field','weight_speed_change','value',Inf), ...
            'complexHeadingChange',struct('field','weight_heading_change','value',1+1i), ...
            'vectorReferenceSpeed',struct('field','weight_reference_speed','value',[1,2]), ...
            'negativeReverse',struct('field','reverse_penalty','value',-1), ...
            'infiniteReverse',struct('field','reverse_penalty','value',Inf))
    end

    methods (TestClassSetup)
        function addSourceAndLoadConfig(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectDir,'matlab','single_vehicle')));
            testCase.VehicleConfig = jsondecode(fileread(fullfile(projectDir, ...
                'config','vehicle_config.json')));
            planner = jsondecode(fileread(fullfile(projectDir, ...
                'config','planner_config.json')));
            if isfield(planner.vhybrid,'goal_connection_radius_scales')
                planner.vhybrid = rmfield(planner.vhybrid,'goal_connection_radius_scales');
            end
            if isfield(planner.vhybrid,'goal_connection_selection')
                planner.vhybrid = rmfield(planner.vhybrid,'goal_connection_selection');
            end
            if isfield(planner.vhybrid,'goal_connection_cost_pruning_enabled')
                planner.vhybrid = rmfield(planner.vhybrid,'goal_connection_cost_pruning_enabled');
            end
            if isfield(planner.vhybrid,'goal_connection_max_turn_rad')
                planner.vhybrid = rmfield(planner.vhybrid,'goal_connection_max_turn_rad');
            end
            planner.vhybrid.goal_connection_distance_m = 40;
            planner.vhybrid.terminal_speed_mps = 0;
            testCase.PlannerConfig = planner;
            map.bounds = [0,0,40,40];
            map.boundary_xy = [0,0;40,0;40,40;0,40;0,0];
            map.centerline_xy = zeros(0,2);
            map.obstacles = cell(0,1);
            testCase.Map = map;
        end
    end

    methods (Test)
        function testDefaultMatchesExplicitUnitRadius(testCase,gear)
            % 缺省参数必须与显式[1]逐节点完全一致，保护Day6基线。
            planner = testCase.PlannerConfig;
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            [defaultSuccess,defaultNodes,defaultStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_radius_scales = 1;
            [unitSuccess,unitNodes,unitStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(defaultSuccess);
            testCase.verifyEqual(unitSuccess,defaultSuccess);
            % 此处有意采用精确比较，检测兼容分支中的任何数值变化。
            testCase.verifyEqual(unitNodes,defaultNodes);
            testCase.verifyEqual(unitStatistics,defaultStatistics);
        end

        function testInvalidRadiusScalesHaveExplicitError(testCase,invalidScales)
            % 溢出样本使用门限内父节点，确保实际计算派生半径。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_radius_scales = invalidScales;
            [parent,goal] = testCase.makeTurningCase(1,testCase.minimumRadius());

            testCase.verifyError(@() vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner), ...
                'VHybrid:InvalidGoalConnectionRadiusScales');
        end

        function testDefaultMatchesExplicitFirstFeasible(testCase,gear)
            % 缺省选择模式必须保持原首条可行连接及统计完全一致。
            planner = testCase.PlannerConfig;
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            parent.g_cost = 7.25;
            [defaultSuccess,defaultNodes,defaultStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_selection = 'first_feasible';
            [explicitSuccess,explicitNodes,explicitStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(defaultSuccess);
            testCase.verifyEqual(explicitSuccess,defaultSuccess);
            % 兼容要求采用精确比较，浮点容差会掩盖原行为变化。
            testCase.verifyEqual(explicitNodes,defaultNodes);
            testCase.verifyEqual(explicitStatistics,defaultStatistics);
            testCase.verifyFalse(defaultStatistics.cost_bound_enabled);
            testCase.verifyEqual(defaultStatistics.cost_bound_pruned,0);
        end

        function testInvalidSelectionHasExplicitError(testCase,invalidSelection)
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_selection = invalidSelection;
            [parent,goal] = testCase.makeTurningCase(1,testCase.minimumRadius());

            testCase.verifyError(@() vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner), ...
                'VHybrid:InvalidGoalConnectionSelection');
        end

        function testDefaultTurnLimitMatchesExplicitInfinity(testCase,gear)
            % Inf和缺省必须逐节点、逐统计完全一致，保护Day6原连接行为。
            planner = testCase.PlannerConfig;
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            [defaultSuccess,defaultNodes,defaultStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_max_turn_rad = Inf;
            [infiniteSuccess,infiniteNodes,infiniteStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(defaultSuccess);
            testCase.verifyEqual(infiniteSuccess,defaultSuccess);
            testCase.verifyEqual(infiniteNodes,defaultNodes);
            testCase.verifyEqual(infiniteStatistics,defaultStatistics);
            testCase.verifyEqual(defaultStatistics.turning_geometry_pruned,0);
        end

        function testExcessiveTurnRejectedBeforeSpeedIntegration(testCase,gear)
            % 起终航向差pi/2；pi/4门限必须在速度积分前拒绝所有几何候选。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_max_turn_rad = pi/4;
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            [success,nodes,statistics] = vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.verifyFalse(success);
            testCase.verifyEmpty(nodes);
            testCase.verifyGreaterThan(statistics.geometric_candidates,0);
            testCase.verifyEqual(statistics.turning_geometry_pruned, ...
                statistics.geometric_candidates);
            testCase.verifyEqual(statistics.reason_counts.excessive_connection_turn, ...
                statistics.geometric_candidates);
            testCase.verifyEqual(statistics.candidates,0);
            testCase.verifyEqual(statistics.boundary_geometry_pruned,0);
            testCase.verifyEqual(statistics.collision_pruned,0);
            testCase.verifyEqual(statistics.constraint_pruned,0);
        end

        function testShortTurnRemainsFeasible(testCase,gear)
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_max_turn_rad = pi/2+1e-8;
            radius = testCase.minimumRadius();
            [parent,goal] = testCase.makeTurningCase(gear,radius);
            [success,nodes,statistics] = vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(success);
            testCase.assertNotEmpty(nodes);
            testCase.verifyEqual(statistics.connection_length_m,radius*pi/2+3, ...
                'AbsTol',1e-8);
            testCase.verifyLessThanOrEqual(sum(abs(diff(unwrap( ...
                [parent.theta,nodes.theta])))),planner.vhybrid.goal_connection_max_turn_rad);
            testCase.verifyNodeDynamics(parent,nodes,planner);
            testCase.verifyTerminalPose(nodes,statistics,goal);
        end

        function testTurnLimitUsesEachCandidateRadius(testCase,gear)
            % 两个半径都有pi/2候选；错误地除以最小半径会多拒绝大半径候选。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            planner.vhybrid.goal_connection_radius_scales = [1,1.5];
            planner.vhybrid.goal_connection_max_turn_rad = pi/2+1e-8;
            radius = testCase.minimumRadius();
            [parent,goal] = testCase.makeTurningCase(gear,1.5*radius);
            motionStart = [parent.x,parent.y,parent.theta+(gear<0)*pi];
            motionGoal = goal(1:3)+[0,0,(gear<0)*pi];
            unitCurves = vhybrid_dubins_candidates(motionStart,motionGoal,radius);
            largeCurves = vhybrid_dubins_candidates(motionStart,motionGoal,1.5*radius);
            unitTurns = arrayfun(@(curve)sum(curve.lengths(curve.types~=0))/radius,unitCurves);
            largeTurns = arrayfun(@(curve)sum(curve.lengths(curve.types~=0))/(1.5*radius),largeCurves);
            expectedPruned = nnz([unitTurns;largeTurns]>planner.vhybrid.goal_connection_max_turn_rad);
            [success,nodes,statistics] = vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(success);
            testCase.assertNotEmpty(nodes);
            testCase.verifyGreaterThan(expectedPruned,0);
            testCase.verifyEqual(statistics.geometric_candidates, ...
                numel(unitCurves)+numel(largeCurves));
            testCase.verifyEqual(statistics.turning_geometry_pruned,expectedPruned);
            testCase.verifyEqual(statistics.reason_counts.excessive_connection_turn,expectedPruned);
            testCase.verifyNodeDynamics(parent,nodes,planner);
            testCase.verifyTerminalPose(nodes,statistics,goal);
        end

        function testInvalidMaximumTurnHasExplicitError(testCase,invalidMaxTurn)
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_max_turn_rad = invalidMaxTurn;
            [parent,goal] = testCase.makeTurningCase(1,testCase.minimumRadius());

            testCase.verifyError(@() vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner), ...
                'VHybrid:InvalidGoalConnectionMaxTurn');
        end

        function testLowestCostDoesNotExceedFirstFeasible(testCase,gear)
            % 在同一父节点和候选集合中比较累计g，不能只比较连接几何长度。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_radius_scales = [1,1.5];
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            parent.g_cost = 7.25;
            planner.vhybrid.goal_connection_selection = 'first_feasible';
            [firstSuccess,firstNodes,firstStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            [lowestSuccess,lowestNodes,lowestStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(firstSuccess);
            testCase.assertTrue(lowestSuccess);
            testCase.assertNotEmpty(firstNodes);
            testCase.assertNotEmpty(lowestNodes);
            testCase.verifyGreaterThanOrEqual(firstNodes(end).g_cost,parent.g_cost);
            testCase.verifyGreaterThanOrEqual(lowestNodes(end).g_cost,parent.g_cost);
            testCase.verifyLessThanOrEqual(lowestNodes(end).g_cost, ...
                firstNodes(end).g_cost+1e-8);
            testCase.verifyNodeDynamics(parent,firstNodes,planner);
            testCase.verifyNodeDynamics(parent,lowestNodes,planner);
            testCase.verifyTerminalPose(firstNodes,firstStatistics,goal);
            testCase.verifyTerminalPose(lowestNodes,lowestStatistics,goal);
        end

        function testLowestCostPruningMatchesExhaustive(testCase,gear)
            % 剪枝必须与完整枚举选中完全相同的真实路径，而非仅代价接近。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_radius_scales = [1,1.5];
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            planner.vhybrid.goal_connection_cost_pruning_enabled = false;
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            parent.g_cost = -17.25;
            [fullSuccess,fullNodes,fullStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_cost_pruning_enabled = true;
            [prunedSuccess,prunedNodes,prunedStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(fullSuccess);
            testCase.assertTrue(prunedSuccess);
            testCase.verifyEqual(prunedNodes,fullNodes);
            testCase.verifyEqual(prunedStatistics.terminal_speed,fullStatistics.terminal_speed);
            testCase.verifyEqual(prunedStatistics.connection_length_m, ...
                fullStatistics.connection_length_m);
            testCase.verifyFalse(fullStatistics.cost_bound_enabled);
            testCase.verifyEqual(fullStatistics.cost_bound_pruned,0);
            testCase.verifyTrue(prunedStatistics.cost_bound_enabled);
            testCase.verifyEqual(prunedStatistics.cost_bound_pruned, ...
                prunedStatistics.cost_bound_integration_pruned);
        end

        function testZeroCostTiePruningKeepsFirstFeasible(testCase,gear)
            % 全零非负权重使后续可行候选在首步达到代价界，显式覆盖剪枝。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            planner.vhybrid.weight_distance = 0;
            planner.vhybrid.weight_speed_change = 0;
            planner.vhybrid.weight_reference_speed = 0;
            planner.vhybrid.weight_heading_change = 0;
            planner.vhybrid.weight_time = 0;
            start = [20-5*gear,20,0,0,0];
            goal = [20+5*gear,20,0,0,0];
            parent = vhybrid_node(start,0,0,0,9,0,zeros(1,5),1,gear,false);
            planner.vhybrid.goal_connection_cost_pruning_enabled = false;
            [fullSuccess,fullNodes,~] = vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_cost_pruning_enabled = true;
            [prunedSuccess,prunedNodes,statistics] = vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(fullSuccess);
            testCase.assertTrue(prunedSuccess);
            testCase.verifyEqual(prunedNodes,fullNodes);
            testCase.verifyEqual(prunedNodes(end).g_cost,parent.g_cost);
            testCase.verifyTrue(statistics.cost_bound_enabled);
            testCase.verifyGreaterThan(statistics.cost_bound_pruned,0);
            testCase.verifyEqual(statistics.cost_bound_pruned, ...
                statistics.cost_bound_integration_pruned);
            testCase.verifyTerminalPose(prunedNodes,statistics,goal);
        end

        function testUnsafeCostsDisablePruning(testCase,unsafeCost)
            % 门限外无需积分；通过公开统计验证每种不安全系数均禁用剪枝。
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            planner.vhybrid.(unsafeCost.field) = unsafeCost.value;
            parent = vhybrid_node([25,25,0,0,0],0,0,0,0,0,zeros(1,5),1,-1,false);
            [success,nodes,statistics] = vhybrid_goal_connection(parent,[100,100,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.verifyFalse(success);
            testCase.verifyEmpty(nodes);
            testCase.verifyFalse(statistics.cost_bound_enabled);
            testCase.verifyEqual(statistics.cost_bound_pruned,0);
        end

        function testInvalidPruningSettingHasExplicitError(testCase,invalidPruning)
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            planner.vhybrid.goal_connection_cost_pruning_enabled = invalidPruning;
            [parent,goal] = testCase.makeTurningCase(1,testCase.minimumRadius());

            testCase.verifyError(@() vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner), ...
                'VHybrid:InvalidGoalConnectionCostPruning');
        end

        function testLargerRadiusRespectsDynamicsAndTerminalPose(testCase,gear)
            planner = testCase.PlannerConfig;
            planner.vhybrid.goal_connection_radius_scales = 1.5;
            planner.vhybrid.goal_connection_max_turn_rad = pi/2+1e-8;
            radius = 1.5*testCase.minimumRadius();
            expectedSteering = atan(testCase.VehicleConfig.dimensions_m.wheelbase/radius);
            [parent,goal] = testCase.makeTurningCase(gear,radius);
            [success,nodes,statistics] = vhybrid_goal_connection(parent,goal, ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(success);
            testCase.assertNotEmpty(nodes);
            turning = abs([nodes.steering_angle]);
            turning = turning(turning>1e-12);
            testCase.assertNotEmpty(turning);
            testCase.verifyEqual(turning,repmat(expectedSteering,size(turning)), ...
                'AbsTol',1e-12);
            testCase.verifyLessThan(expectedSteering, ...
                min(testCase.VehicleConfig.steering.maximum_steer_rad, ...
                planner.dynamics.delta_max_rad));
            testCase.verifyEqual(statistics.connection_length_m,radius*pi/2+3, ...
                'AbsTol',1e-8);
            testCase.verifyEqual([nodes(end).x,nodes(end).y,nodes(end).v], ...
                goal([1,2,4]),'AbsTol',1e-8);
            testCase.verifyLessThanOrEqual(abs(testCase.wrapAngle( ...
                nodes(end).theta-goal(3))),1e-8);
            testCase.verifyEqual(statistics.terminal_speed,0,'AbsTol',1e-8);
            testCase.verifyGreaterThan(nodes(1).acceleration,0);
            testCase.verifyGreaterThan(nodes(1).v,0);
            testCase.verifyGreaterThan(abs(nodes(1).travelled_distance),0);
            testCase.verifyNodeDynamics(parent,nodes,planner);
        end

        function testAllRadiiAreComparedByLength(testCase,gear)
            % 较大半径列在首位也不能抢先返回更长连接。
            planner = testCase.PlannerConfig;
            [parent,goal] = testCase.makeTurningCase(gear,testCase.minimumRadius());
            planner.vhybrid.goal_connection_radius_scales = 1;
            [unitSuccess,unitNodes,unitStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);
            planner.vhybrid.goal_connection_radius_scales = [1.5,1];
            [mixedSuccess,mixedNodes,mixedStatistics] = vhybrid_goal_connection( ...
                parent,goal,testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(unitSuccess);
            testCase.assertTrue(mixedSuccess);
            testCase.verifyEqual(mixedStatistics.connection_length_m, ...
                unitStatistics.connection_length_m,'AbsTol',1e-8);
            testCase.verifyEqual(mixedNodes,unitNodes);
        end
    end

    methods (Access = private)
        function radius = minimumRadius(testCase)
            delta = min(testCase.VehicleConfig.steering.maximum_steer_rad, ...
                testCase.PlannerConfig.dynamics.delta_max_rad);
            radius = testCase.VehicleConfig.dimensions_m.wheelbase/tan(delta);
        end

        function [parent,goal] = makeTurningCase(~,gear,radius)
            % 两段pi/4圆弧之间留3m直线，避免零长度几何段退化。
            % 倒车时运动切向增加pi，车身末航向仍为pi/2。
            startXY = [17.5,17.5]-7.5*gear;
            start = [startXY,0,0,0];
            offset = gear*(radius+3/sqrt(2));
            goal = [startXY+[offset,offset],pi/2,0,0];
            parent = vhybrid_node(start,0,0,0,0,0,zeros(1,5),1,gear,false);
        end

        function verifyNodeDynamics(testCase,parent,nodes,planner)
            % 控制约束、时刻、距离和静态矩形碰撞独立于连接内部验收。
            dyn = planner.dynamics;
            deltaMax = min(testCase.VehicleConfig.steering.maximum_steer_rad, ...
                dyn.delta_max_rad);
            testCase.verifyGreaterThanOrEqual([nodes.v],dyn.v_min_mps-1e-8);
            testCase.verifyLessThanOrEqual([nodes.v],dyn.v_max_mps+1e-8);
            testCase.verifyGreaterThanOrEqual([nodes.acceleration],dyn.a_min_mps2-1e-8);
            testCase.verifyLessThanOrEqual([nodes.acceleration],dyn.a_max_mps2+1e-8);
            testCase.verifyLessThanOrEqual(abs([nodes.steering_angle]),deltaMax+1e-12);
            testCase.verifyEqual([nodes.gear],repmat(parent.gear,1,numel(nodes)));
            testCase.verifyFalse(any([nodes.is_gear_change]));
            previous = [parent.x,parent.y,parent.theta,parent.v,parent.t];
            for k = 1:numel(nodes)
                actual = [nodes(k).x,nodes(k).y,nodes(k).theta,nodes(k).v,nodes(k).t];
                dt = actual(5)-previous(5);
                testCase.assertGreaterThan(dt,0);
                [replayed,distance,~,valid] = summon_vehicle_dynamic_gear( ...
                    previous,nodes(k).steering_angle,nodes(k).acceleration,dt, ...
                    testCase.VehicleConfig,planner,nodes(k).gear);
                testCase.assertTrue(valid);
                testCase.verifyEqual(actual([1,2,4,5]),replayed([1,2,4,5]),'AbsTol',1e-8);
                testCase.verifyLessThanOrEqual(abs(testCase.wrapAngle( ...
                    actual(3)-replayed(3))),1e-8);
                testCase.verifyEqual(nodes(k).travelled_distance,distance,'AbsTol',1e-8);
                testCase.verifyFalse(day6_static_collision_check(actual(1:3), ...
                    testCase.Map,testCase.VehicleConfig));
                previous = actual;
            end
        end

        function verifyTerminalPose(testCase,nodes,statistics,goal)
            % 两种选择模式都必须以真实积分达到相同目标位姿并停车。
            testCase.verifyEqual([nodes(end).x,nodes(end).y,nodes(end).v], ...
                goal([1,2,4]),'AbsTol',1e-8);
            testCase.verifyLessThanOrEqual(abs(testCase.wrapAngle( ...
                nodes(end).theta-goal(3))),1e-8);
            testCase.verifyEqual(statistics.terminal_speed,goal(4),'AbsTol',1e-8);
        end

        function angle = wrapAngle(~,angle)
            angle = mod(angle+pi,2*pi)-pi;
        end
    end
end
