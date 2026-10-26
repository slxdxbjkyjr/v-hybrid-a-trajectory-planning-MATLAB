classdef test_vhybrid_refinement < matlab.unittest.TestCase
    %TEST_VHYBRID_REFINEMENT 首解后固定预算细化的公开接口回归。
    % 缺省/零预算保持首解路径；正预算保留完整运动学验收后的最低累计g。
    % 节点上限、Open耗尽及动态起点冲突不得丢失解或伪造可行解。

    properties
        VehicleConfig
        Map
        PlannerConfig
        FirstPath
    end

    properties (TestParameter)
        invalidBudget = struct('negative',-1,'nanValue',NaN, ...
            'infinite',Inf,'vector',[0,1],'text','1', ...
            'complexValue',1i,'emptyValue',[]);
    end

    methods (TestClassSetup)
        function loadConfigAndFirstSolution(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectDir,'matlab','single_vehicle')));
            testCase.VehicleConfig = jsondecode(fileread(fullfile(projectDir, ...
                'config','vehicle_config.json')));
            testCase.Map = summon_map(jsondecode(fileread(fullfile(projectDir, ...
                'config','map_config.json'))));
            planner = jsondecode(fileread(fullfile(projectDir, ...
                'config','planner_config.json')));
            planner.vhybrid.reverse_enabled = false;
            planner.vhybrid.initial_gear = 1;
            planner.vhybrid.initial_gear_mode = 'fixed';
            % 不设置可选预算，直接走旧有首解路径。
            testCase.PlannerConfig = planner;
            testCase.FirstPath = summon_vhybrid_astar([10,10,0,0,0], ...
                [14,10,0,0,0],testCase.Map,testCase.VehicleConfig,planner);
            testCase.assertTrue(testCase.FirstPath.valid);
            testCase.assertTrue(testCase.FirstPath.kinematic_validation.valid);
        end
    end

    methods (Test)
        function zeroBudgetMatchesDefaultPathAndCounters(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = 0;
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(actual.valid);
            testCase.verifyEqual(testCase.pathSamples(actual), ...
                testCase.pathSamples(testCase.FirstPath));
            testCase.verifyEqual(actual.search_nodes,testCase.FirstPath.search_nodes);
            testCase.verifyEqual(testCase.deterministicStats(actual), ...
                testCase.deterministicStats(testCase.FirstPath));
            testCase.verifyEqual(actual.search_statistics.refinement_stop_reason,'disabled');
        end

        function positiveBudgetKeepsValidatedNonWorseIncumbent(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = 0.1;
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);
            replay = vhybrid_validate_trajectory(actual,testCase.Map, ...
                testCase.VehicleConfig,planner);

            testCase.assertTrue(actual.valid);
            testCase.verifyTrue(replay.valid);
            testCase.verifyEqual(actual.error_code,uint8(0));
            testCase.verifyTrue(isfinite(actual.search_statistics.first_solution_time_s));
            testCase.verifyLessThanOrEqual(actual.search_statistics.final_solution_cost, ...
                actual.search_statistics.first_solution_cost);
            testCase.verifyEqual(actual.v(end),0,'AbsTol',1e-9);
            testCase.verifyEqual(actual.search_statistics.path_nodes,numel(actual.x));
            testCase.verifyGreaterThanOrEqual(actual.search_statistics.elapsed_sec, ...
                actual.search_statistics.first_solution_time_s);
        end

        function tinyBudgetStillReturnsFirstCompleteSolution(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = realmin;
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(actual.valid);
            testCase.verifyTrue(actual.kinematic_validation.valid);
            testCase.verifyEqual(testCase.pathSamples(actual), ...
                testCase.pathSamples(testCase.FirstPath));
            testCase.verifyEqual(actual.search_statistics.refinement_stop_reason, ...
                'refinement_time_limit');
            testCase.verifyEqual(actual.search_statistics.solution_improvements,0);
            testCase.verifyEqual(actual.search_statistics.final_solution_cost, ...
                actual.search_statistics.first_solution_cost);
        end

        function nodeLimitReturnsExistingValidatedIncumbent(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = 60;
            % 根节点的连接恰好用尽预算，但连接后的父节点仍须扩展。
            planner.vhybrid.max_search_nodes = numel(testCase.FirstPath.search_nodes);
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(actual.valid);
            testCase.verifyTrue(actual.kinematic_validation.valid);
            testCase.verifyEqual(actual.error_code,uint8(0));
            testCase.verifyEqual(actual.search_statistics.refinement_stop_reason,'node_limit');
            testCase.verifyGreaterThan(actual.search_statistics.sampled_controls,0);
            testCase.verifyLessThanOrEqual(numel(actual.search_nodes), ...
                planner.vhybrid.max_search_nodes);
            testCase.verifyEqual(testCase.pathSamples(actual), ...
                testCase.pathSamples(testCase.FirstPath));
        end

        function emptyOpenReturnsExistingValidatedIncumbent(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = 60;
            % 零速节点不能执行负加速度；终点连接独立使用既有加减速限制。
            planner.vhybrid.control_acceleration_samples_mps2 = -2;
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.assertTrue(actual.valid);
            testCase.verifyTrue(actual.kinematic_validation.valid);
            testCase.verifyEqual(actual.search_statistics.refinement_stop_reason,'open_exhausted');
            testCase.verifyEqual(actual.search_statistics.generated_nodes,0);
            testCase.verifyGreaterThan(actual.search_statistics.sampled_controls,0);
        end

        function noSolutionKeepsExplicitNodeLimitFailure(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = 60;
            planner.vhybrid.goal_connection_enabled = false;
            planner.vhybrid.max_search_nodes = 1;
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner);

            testCase.verifyFalse(actual.valid);
            testCase.verifyEqual(actual.error_code,uint8(5));
            testCase.verifyTrue(isnan(actual.search_statistics.first_solution_time_s));
            testCase.verifyTrue(isnan(actual.search_statistics.final_solution_cost));
            testCase.verifyEqual(actual.search_statistics.refinement_stop_reason,'no_solution');
        end

        function refinementDoesNotBypassDynamicStartCollision(testCase)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = 0.1;
            parked = dynamic_vehicle_obstacle([10,10,0,0,0;10,10,0,0,60], ...
                testCase.VehicleConfig,'blocked_vehicle',planner);
            actual = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.VehicleConfig,planner,[],parked);

            testCase.verifyFalse(actual.valid);
            testCase.verifyEqual(actual.error_code,uint8(9));
            testCase.verifyTrue(actual.dynamic_validation.collision);
            testCase.verifyTrue(isnan(actual.search_statistics.first_solution_time_s));
        end

        function invalidBudgetHasExplicitError(testCase,invalidBudget)
            planner = testCase.PlannerConfig;
            planner.vhybrid.solution_refinement_time_s = invalidBudget;

            testCase.verifyError(@() summon_vhybrid_astar([10,10,0,0,0], ...
                [14,10,0,0,0],testCase.Map,testCase.VehicleConfig,planner), ...
                'VHybridAStar:InvalidRefinementTime');
        end
    end

    methods (Static,Access=private)
        function samples = pathSamples(path)
            samples = [path.x,path.y,path.theta,path.v,path.t, ...
                path.acceleration,path.steering_angle,path.travelled_distance, ...
                path.gear,double(path.is_gear_change)];
        end

        function stats = deterministicStats(path)
            % 实际墙钟耗时不属于确定性回归比较。
            stats = rmfield(path.search_statistics, ...
                {'elapsed_sec','first_solution_time_s'});
        end
    end
end
