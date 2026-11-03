classdef test_lateral_search_integration < matlab.unittest.TestCase
    %TEST_LATERAL_SEARCH_INTEGRATION 验证搜索动作拒绝和统计汇总。

    properties
        Vehicle
        Map
        Planner
    end

    methods (TestClassSetup)
        function addSourceAndLoadConfiguration(testCase)
            project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(project,'matlab','single_vehicle')));
            testCase.Vehicle = jsondecode(fileread(fullfile(project, ...
                'config','vehicle_config_day7.json')));
            testCase.Map = summon_map(jsondecode(fileread(fullfile(project, ...
                'config','map_config_day7.json'))));
            testCase.Planner = jsondecode(fileread(fullfile(project, ...
                'config','planner_config_day7_lateral.json')));
        end
    end

    methods (Test)
        function highSpeedBrakingTurnIsRejectedAtEntry(testCase)
            planner = testCase.Planner;
            delta = atan(0.278);
            planner.vhybrid.control_acceleration_samples_mps2 = -6;
            planner.vhybrid.control_steering_samples_rad = delta;
            parent = vhybrid_node([10,10,0,8,0],0,0,0,0,0,[20,20,12,32,0],1,1,false);

            [children,statistics] = vhybrid_expand_node(parent,testCase.Map, ...
                testCase.Vehicle,planner,[15,10,0,0,0]);

            testCase.verifyEmpty(children);
            testCase.verifyEqual(statistics.lateral_acceleration_pruned,1);
            testCase.verifyEqual(statistics.constraint_pruned,1);
            testCase.verifyEqual(statistics.collision_pruned,0);
            testCase.verifyEqual(statistics.reason_counts.entry_lateral_acceleration_exceeded,1);
            testCase.verifyLessThanOrEqual(5^2*0.1,4.9);
            testCase.verifyGreaterThan(8^2*0.1,4.9);
        end

        function acceptedExpansionEdgesMeetLimit(testCase)
            planner = testCase.Planner;
            delta = atan(0.278);
            planner.vhybrid.control_acceleration_samples_mps2 = [-3,0,3];
            planner.vhybrid.control_steering_samples_rad = [-delta,0,delta];
            parent = vhybrid_node([10,10,0,2,0],0,0,0,0,0,[20,20,12,8,0],1,1,false);

            [children,statistics] = vhybrid_expand_node(parent,testCase.Map, ...
                testCase.Vehicle,planner,[15,10,0,0,0]);
            peaks = arrayfun(@(child) max(abs([parent.v,child.v]))^2* ...
                abs(tan(child.steering_angle)/testCase.Vehicle.dimensions_m.wheelbase),children);

            testCase.verifyNotEmpty(children);
            testCase.verifyEqual(statistics.lateral_acceleration_pruned,0);
            testCase.verifyLessThanOrEqual(max(peaks), ...
                planner.dynamics.max_lateral_acceleration_mps2 + ...
                planner.dynamics.lateral_acceleration_tolerance_mps2);
        end

        function missingEnableFieldKeepsLegacyExpansion(testCase)
            planner = testCase.Planner;
            planner.dynamics = rmfield(planner.dynamics, ...
                'lateral_acceleration_constraint_enabled');
            planner.vhybrid.control_acceleration_samples_mps2 = -6;
            planner.vhybrid.control_steering_samples_rad = atan(0.278);
            parent = vhybrid_node([10,10,0,8,0],0,0,0,0,0,[20,20,12,32,0],1,1,false);

            [children,statistics] = vhybrid_expand_node(parent,testCase.Map, ...
                testCase.Vehicle,planner,[15,10,0,0,0]);

            testCase.verifyNotEmpty(children);
            testCase.verifyEqual(statistics.lateral_acceleration_pruned,0);
            testCase.verifyEqual(statistics.collision_pruned,0);
            testCase.verifyEqual(children(1).v,5,'AbsTol',1e-12);
        end

        function searchReportsSourceCountsAndFinalPathPeak(testCase)
            planner = testCase.Planner;
            planner.vhybrid.max_search_nodes = 20000;
            planner.vhybrid.max_search_time_s = 10;
            path = summon_vhybrid_astar([10,10,0,0,0],[14,10,0,0,0], ...
                testCase.Map,testCase.Vehicle,planner);
            statistics = path.search_statistics;
            expected_peak = max(max(abs(path.v(1:end-1)),abs(path.v(2:end))).^2 .* ...
                abs(tan(path.steering_angle(2:end))/testCase.Vehicle.dimensions_m.wheelbase));

            testCase.verifyTrue(path.valid);
            testCase.verifyEqual(statistics.lateral_acceleration_pruned, ...
                statistics.lateral_acceleration_pruned_nodes + ...
                statistics.goal_connection_lateral_acceleration_pruned);
            testCase.verifyEqual(statistics.max_path_lateral_acceleration_mps2, ...
                expected_peak,'AbsTol',1e-10);
            testCase.verifyLessThanOrEqual(statistics.max_path_lateral_acceleration_mps2, ...
                planner.dynamics.max_lateral_acceleration_mps2 + ...
                planner.dynamics.lateral_acceleration_tolerance_mps2);
        end

        function searchSeparatesNodeAndGoalConnectionRejections(testCase)
            planner = testCase.Planner;
            planner.vhybrid.goal_connection_max_turn_rad = 10;
            planner.vhybrid.max_search_nodes = 1;
            planner.vhybrid.max_search_time_s = 10;
            planner.vhybrid.initial_gear_mode = 'fixed';
            planner.vhybrid.initial_gear = 1;
            planner.vhybrid.control_acceleration_samples_mps2 = [-6,0];
            delta = atan(0.278);
            planner.vhybrid.control_steering_samples_rad = [-delta,0,delta];
            path = summon_vhybrid_astar([10,10,0,8,0],[15,14,pi/2,0,0], ...
                testCase.Map,testCase.Vehicle,planner);
            statistics = path.search_statistics;

            testCase.verifyEqual(path.error_code,uint8(5));
            testCase.verifyGreaterThan(statistics.lateral_acceleration_pruned_nodes,0);
            testCase.verifyGreaterThan(statistics.goal_connection_lateral_acceleration_pruned,0);
            testCase.verifyEqual(statistics.lateral_acceleration_pruned, ...
                statistics.lateral_acceleration_pruned_nodes + ...
                statistics.goal_connection_lateral_acceleration_pruned);
            testCase.verifyEqual(statistics.reason_counts.entry_lateral_acceleration_exceeded, ...
                statistics.lateral_acceleration_pruned);
            testCase.verifyEqual(statistics.collision_pruned_nodes,0);
            testCase.verifyEqual(statistics.goal_connection_collision_pruned,0);
        end
    end
end
