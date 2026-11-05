classdef test_vhybrid_core < matlab.unittest.TestCase
    %TEST_VHYBRID_CORE 检查五维搜索和可按车辆运动学重放的终点连接。
    % 测试输入来自项目集中配置；输出为 MATLAB 单元测试的通过/失败结果。
    % 重点防止位置、航向和速度分别插值，以及末端强制覆盖姿态。

    properties
        VehicleConfig
        Map
        PlannerConfig
        DemoPath
    end

    methods (TestClassSetup)
        function addSourceAndLoadConfig(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectDir, 'matlab', 'single_vehicle')));
            testCase.VehicleConfig = jsondecode(fileread(fullfile(projectDir, ...
                'config', 'vehicle_config.json')));
            testCase.Map = summon_map(jsondecode(fileread(fullfile(projectDir, ...
                'config', 'map_config.json'))));
            testCase.PlannerConfig = jsondecode(fileread(fullfile(projectDir, ...
                'config', 'planner_config.json')));
            % 本套件保留Day4前进扩展的基线语义；倒车由新套件独立覆盖。
            testCase.PlannerConfig.vhybrid.reverse_enabled = false;
        end
    end

    methods (Test)
        function testNodeContainsFiveDimensionalState(testCase)
            node = vhybrid_node([1,2,0.1,0.5,1.5], 1, 0.2, 3, 4, 5, [1,2,3,4,5], 6);
            testCase.verifyEqual([node.x,node.y,node.theta,node.v,node.t], [1,2,0.1,0.5,1.5]);
            testCase.verifyEqual([node.acceleration,node.steering_angle,node.parent_id], [1,0.2,3]);
            testCase.verifyEqual([node.g_cost,node.h_cost,node.f_cost], [4,5,9]);
            testCase.verifyEqual([node.x_index,node.y_index,node.yaw_index,node.velocity_index,node.time_index], [1,2,3,4,5]);
        end

        function testClosedSetPreservesTimeAndVelocity(testCase)
            closed = vhybrid_closed_set();
            node_a = vhybrid_node([1,2,0,0.5,1],0,0,0,0,0,[2,4,6,2,2],1);
            node_b = vhybrid_node([1,2,0,1.0,1.5],0,0,0,0,0,[2,4,6,4,3],2);
            closed.add(node_a);
            testCase.verifyTrue(closed.contains(node_a));
            testCase.verifyFalse(closed.contains(node_b));
        end

        function testExpansionSamplesConfiguredControls(testCase)
            % 零速父节点须采样所有配置控制，并能生成低速加速节点。
            parent = vhybrid_node([3,3,0,0,0],0,0,0,0,0,[6,6,12,0,0],1);
            [children, statistics] = vhybrid_expand_node(parent, testCase.Map, ...
                testCase.VehicleConfig, testCase.PlannerConfig, [24,5,0,0,0]);
            cfg = testCase.PlannerConfig.vhybrid;
            expected_controls = numel(cfg.control_acceleration_samples_mps2) ...
                * numel(cfg.control_steering_samples_rad);
            testCase.verifyEqual(statistics.sampled, expected_controls);
            testCase.verifyGreaterThan(statistics.valid, 0);
            testCase.verifyGreaterThanOrEqual(numel(children), 3);
            testCase.verifyTrue(any(abs([children.v]-0.5)<1e-12));
            testCase.verifyTrue(any(abs([children.v]-1.0)<1e-12));
            testCase.verifyGreaterThanOrEqual(min([children.v]), 0);
        end

        function testExpansionUsesDifferentTimeIndices(testCase)
            parent = vhybrid_node([3,3,0,0.5,0],0,0,0,0,0,[6,6,12,2,0],1);
            [children, ~] = vhybrid_expand_node(parent, testCase.Map, ...
                testCase.VehicleConfig, testCase.PlannerConfig, [24,5,0,0,0]);
            testCase.verifyGreaterThan(numel(unique([children.time_index])), 0);
            testCase.verifyEqual([children.t], repmat(testCase.PlannerConfig.vhybrid.time_step_s, 1, numel(children)), 'AbsTol', 1e-12);
        end

        function testCostContainsRequiredTerms(testCase)
            parent = vhybrid_node([3,3,0,0.5,0],0,0,0,0,0,[6,6,12,2,0],1);
            child_state = [3.5,3.0,0.1,0.7,0.5];
            [g_cost,h_cost,f_cost,terms] = vhybrid_cost(parent, child_state, ...
                [10,3,0,0,0], testCase.PlannerConfig, 0.5, 0.1);
            testCase.verifyGreaterThan(g_cost, 0);
            testCase.verifyGreaterThan(h_cost, 0);
            testCase.verifyEqual(f_cost, g_cost+h_cost, 'AbsTol', 1e-12);
            testCase.verifyEqual([terms.distance,terms.speed_change,terms.heading_change,terms.goal_distance,terms.time], ...
                [0.5,0.2,0.1,6.5,0.5], 'AbsTol', 1e-12);
            testCase.verifyEqual(terms.reference_speed, ...
                abs(child_state(4)-testCase.PlannerConfig.vhybrid.reference_speed_mps), 'AbsTol', 1e-12);
        end

        function testCostCanSkipUnusedHeuristic(testCase)
            parent = vhybrid_node([3,3,0,0.5,0],0,0,0,0,0,[6,6,12,2,0],1);
            child_state = [3.5,3.0,0.1,0.7,0.5];
            goal = [10,3,0,0,0];
            [default_g,default_h,default_f] = vhybrid_cost(parent,child_state, ...
                goal,testCase.PlannerConfig,0.5,0);
            [fast_g,fast_h,fast_f,terms] = vhybrid_cost(parent,child_state, ...
                goal,testCase.PlannerConfig,0.5,0,1,false,false);

            testCase.verifyEqual(fast_g,default_g,'AbsTol',1e-12);
            testCase.verifyEqual(default_f,default_g+default_h,'AbsTol',1e-12);
            testCase.verifyEqual(fast_h,0);
            testCase.verifyEqual(fast_f,fast_g,'AbsTol',1e-12);
            testCase.verifyEqual(terms.heuristic_method,'disabled');
        end

        function testOpenSetReturnsMinimumCost(testCase)
            open = vhybrid_open_set();
            node_a = vhybrid_node([0,0,0,0,0],0,0,0,3,2,[0,0,0,0,0],1);
            node_b = vhybrid_node([0,0,0,0,0],0,0,0,1,1,[0,0,0,0,0],2);
            open.push(node_a); open.push(node_b);
            [node,valid] = open.pop_min();
            testCase.verifyTrue(valid);
            testCase.verifyEqual(node.node_id, 2);
            testCase.verifyEqual(open.count(), 1);
        end

        function testDemoReturnsPathOrExplicitFailure(testCase)
            path = testCase.cachedDemo();
            testCase.verifyTrue(isfield(path,'search_statistics'));
            testCase.verifyTrue(path.valid || path.error_code ~= 0);
            testCase.verifyGreaterThan(path.search_statistics.expanded_nodes, 0);
        end

        function testOpenSetKeepsLowerCostForSameState(testCase)
            open = vhybrid_open_set();
            indices = [1,2,3,4,5];
            expensive = vhybrid_node([0,0,0,0,0],0,0,0,5,2,indices,1);
            cheaper = vhybrid_node([0,0,0,0,0],0,0,0,2,1,indices,2);
            testCase.verifyTrue(open.push_or_update(expensive));
            testCase.verifyTrue(open.push_or_update(cheaper));
            testCase.verifyFalse(open.push_or_update(expensive));
            [node, valid] = open.pop_min();
            testCase.verifyTrue(valid);
            testCase.verifyEqual(node.node_id, 2);
            testCase.verifyEqual(open.count(), 1);
        end

        function testOpenSetCanBeReusedAfterRemovingLastNode(testCase)
            open = vhybrid_open_set();
            first = vhybrid_node([0,0,0,0,0],0,0,0,2,1,[0,0,0,0,0],1);
            second = vhybrid_node([1,0,0,0,1],0,0,0,1,1,[1,0,0,0,1],2);
            open.push(first);
            [~, validFirst] = open.pop_min();
            testCase.verifyTrue(validFirst);
            testCase.verifyTrue(open.is_empty());
            open.push(second);
            [node, validSecond] = open.pop_min();
            testCase.verifyTrue(validSecond);
            testCase.verifyEqual(node.node_id, 2);
            testCase.verifyTrue(open.is_empty());
        end

        function testGoalConnectionEndsAtZeroSpeed(testCase)
            parent = vhybrid_node([20,5,0,1,0],0,0,0,0,0,[40,10,12,4,0],1);
            goal = [24,5,0,0,0];
            [success, nodes, statistics] = vhybrid_goal_connection(parent, goal, ...
                testCase.Map, testCase.VehicleConfig, testCase.PlannerConfig);
            testCase.assertTrue(success);
            testCase.verifyNotEmpty(nodes);
            testCase.verifyEqual([nodes(end).x,nodes(end).y,nodes(end).theta,nodes(end).v], ...
                goal(1:4), 'AbsTol', 1e-8);
            testCase.verifyEqual(statistics.terminal_speed, 0, 'AbsTol', 1e-12);
            testCase.verifyNodeDynamics(parent, nodes, testCase.Map);
        end

        function testStationaryVehicleAcceleratesBeforeMoving(testCase)
            parent = vhybrid_node([20,5,0,0,0],0,0,0,0,0,[40,10,12,0,0],1);
            [success, nodes] = vhybrid_goal_connection(parent, [24,5,0,0,0], ...
                testCase.Map, testCase.VehicleConfig, testCase.PlannerConfig);
            testCase.assertTrue(success);
            testCase.verifyGreaterThan(nodes(1).acceleration, 0);
            testCase.verifyGreaterThan(nodes(1).v, 0);
            testCase.verifyGreaterThan(nodes(1).travelled_distance, 0);
            testCase.verifyEqual(nodes(end).v, 0, 'AbsTol', 1e-8);
            testCase.verifyNodeDynamics(parent, nodes, testCase.Map);
        end

        function testCurvedConnectionReachesGoalWithoutHeadingSnap(testCase)
            parent = vhybrid_node([12,7,0,1,0],0,0,0,0,0,[24,14,12,4,0],1);
            goal = [17.6,12.6,pi/2,0,0];
            [success, nodes] = vhybrid_goal_connection(parent, goal, ...
                testCase.Map, testCase.VehicleConfig, testCase.PlannerConfig);
            testCase.assertTrue(success);
            testCase.verifyEqual([nodes(end).x,nodes(end).y,nodes(end).theta,nodes(end).v], ...
                goal(1:4), 'AbsTol', 1e-8);
            testCase.verifyGreaterThan(max(abs([nodes.steering_angle])), 0);
            testCase.verifyNodeDynamics(parent, nodes, testCase.Map);
        end

        function testConnectionWrapsHeadingContinuouslyAtPi(testCase)
            parent = vhybrid_node([24,12,pi-0.08,1,0],0,0,0,0,0,[48,24,24,4,0],1);
            goal = [20,12,-pi+0.08,0,0];
            [success, nodes] = vhybrid_goal_connection(parent, goal, ...
                testCase.Map, testCase.VehicleConfig, testCase.PlannerConfig);
            testCase.assertTrue(success);
            testCase.verifyEqual([nodes(end).x,nodes(end).y], goal(1:2), 'AbsTol', 1e-8);
            testCase.verifyLessThanOrEqual(abs(testCase.wrapAngle(nodes(end).theta-goal(3))), 1e-8);
            testCase.verifyNodeDynamics(parent, nodes, testCase.Map);
        end

        function testInsufficientBrakingDistanceIsRejectedInNarrowLane(testCase)
            narrowMap = testCase.Map;
            narrowMap.bounds = [0,3.75,30,6.25];
            narrowMap.boundary_xy = [0,3.75;30,3.75;30,6.25;0,6.25;0,3.75];
            parent = vhybrid_node([20,5,0,5,0],0,0,0,0,0,[40,10,12,20,0],1);
            [success, nodes] = vhybrid_goal_connection(parent, [20.1,5,0,0,0], ...
                narrowMap, testCase.VehicleConfig, testCase.PlannerConfig);
            testCase.verifyFalse(success);
            testCase.verifyEmpty(nodes);
        end

        function testDemoFinalHeadingMeetsRequestedGoal(testCase)
            path = testCase.cachedDemo();
            testCase.assertTrue(path.valid);
            testCase.verifyEqual(path.error_code, uint8(0));
            testCase.verifyLessThanOrEqual(path.final_position_error_m, 1e-8);
            testCase.verifyLessThanOrEqual(path.final_heading_error_rad, 1e-8);
            testCase.verifyLessThanOrEqual(path.final_speed_error_mps, 1e-8);
            testCase.verifyEqual(path.v(end), 0, 'AbsTol', 1e-8);
            testCase.verifyTrue(path.kinematic_validation.valid);
        end

        function testDemoStartsFromRestAndAcceleratesPhysically(testCase)
            % 起点必须真实静止，首个移动状态须由正加速度和原运动学模型产生。
            path = testCase.cachedDemo();
            testCase.assertTrue(path.valid);
            testCase.verifyEqual(testCase.PlannerConfig.vhybrid.initial_speed_mps, ...
                0, 'AbsTol', 1e-12);
            testCase.verifyEqual(path.v(1), 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(path.t(1), 0, 'AbsTol', 1e-12);
            moving = find(path.travelled_distance > 1e-10, 1, 'first');
            testCase.assertNotEmpty(moving);
            testCase.assertGreaterThan(moving, 1);
            testCase.verifyGreaterThan(path.acceleration(moving), 0);
            testCase.verifyGreaterThan(path.v(moving), 0);
            previous = [path.x(moving-1),path.y(moving-1),path.theta(moving-1), ...
                path.v(moving-1),path.t(moving-1)];
            actual = [path.x(moving),path.y(moving),path.theta(moving), ...
                path.v(moving),path.t(moving)];
            testCase.verifyEdge(previous,actual,path.acceleration(moving), ...
                path.steering_angle(moving),path.travelled_distance(moving),testCase.Map);
        end

        function testDemoEveryEdgeMatchesVehicleDynamics(testCase)
            path = testCase.cachedDemo();
            testCase.assertTrue(path.valid);
            testCase.verifyPathDynamics(path);
            testCase.verifyLessThanOrEqual(path.kinematic_validation.max_position_residual_m, 1e-8);
            testCase.verifyLessThanOrEqual(path.kinematic_validation.max_heading_residual_rad, 1e-8);
            testCase.verifyLessThanOrEqual(path.kinematic_validation.max_speed_residual_mps, 1e-8);
            testCase.verifyEqual(path.kinematic_validation.collision_count, 0);
            testCase.verifyEqual(path.kinematic_validation.constraint_failures, 0);
        end
    end

    methods (Access = private)
        function path = cachedDemo(testCase)
            %CACHEDEMO 同一轮测试只运行一次车辆 2 搜索，不写图像输出。
            if isempty(testCase.DemoPath)
                testCase.DemoPath = run_vhybrid_astar_demo(2, false);
            end
            path = testCase.DemoPath;
        end

        function verifyNodeDynamics(testCase, parent, nodes, map)
            %VERIFYNODEDYNAMICS 对连接的每条边使用已存在的运动学模型重放。
            previous = [parent.x,parent.y,parent.theta,parent.v,parent.t];
            for k = 1:numel(nodes)
                actual = [nodes(k).x,nodes(k).y,nodes(k).theta,nodes(k).v,nodes(k).t];
                testCase.verifyEdge(previous, actual, nodes(k).acceleration, ...
                    nodes(k).steering_angle, nodes(k).travelled_distance, map);
                previous = actual;
            end
        end

        function verifyPathDynamics(testCase, path)
            %VERIFYPATHDYNAMICS 独立验证整条回溯路径，避免仅依赖运行时报告。
            for k = 2:numel(path.x)
                previous = [path.x(k-1),path.y(k-1),path.theta(k-1),path.v(k-1),path.t(k-1)];
                actual = [path.x(k),path.y(k),path.theta(k),path.v(k),path.t(k)];
                testCase.verifyEdge(previous, actual, path.acceleration(k), ...
                    path.steering_angle(k), path.travelled_distance(k), testCase.Map);
            end
        end

        function verifyEdge(testCase, previous, actual, acceleration, steering, distance, map)
            %VERIFYEDGE 输入父/子五维状态与控制，验证位置、航向、速度及碰撞。
            dynamics = testCase.PlannerConfig.dynamics;
            wheelbase = testCase.VehicleConfig.dimensions_m.wheelbase;
            dt = actual(5)-previous(5);
            testCase.assertGreaterThan(dt, 0);
            [replayed, travelled, ~, valid] = summon_vehicle_dynamic( ...
                previous, steering, acceleration, dt, wheelbase, ...
                dynamics.v_max_mps, dynamics.v_min_mps, ...
                dynamics.a_min_mps2, dynamics.a_max_mps2, dynamics.delta_max_rad);
            testCase.assertTrue(valid);
            testCase.verifyEqual(actual([1,2,4,5]), replayed([1,2,4,5]), 'AbsTol', 1e-8);
            testCase.verifyLessThanOrEqual(abs(testCase.wrapAngle(actual(3)-replayed(3))), 1e-8);
            testCase.verifyEqual(distance, travelled, 'AbsTol', 1e-8);
            headingLimit = travelled*tan(dynamics.delta_max_rad)/wheelbase;
            testCase.verifyLessThanOrEqual(abs(testCase.wrapAngle(actual(3)-previous(3))), ...
                headingLimit+1e-8);
            [collision, ~] = summon_collision_check(actual(1:3), map, testCase.VehicleConfig);
            testCase.verifyFalse(collision);
        end

        function angle = wrapAngle(~, angle)
            %WRAPANGLE 比较跨越正负 pi 的航向，避免把等价角误判为突变。
            angle = mod(angle+pi,2*pi)-pi;
        end
    end
end
