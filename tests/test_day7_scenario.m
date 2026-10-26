classdef test_day7_scenario < matlab.unittest.TestCase
    %TEST_DAY7_SCENARIO 验收 Day7 四车配置、地图和起终点安全边界。
    % 测试只读取 Day7 副本，不运行耗时的四车搜索，也不修改 Day6 配置。

    properties
        ProjectRoot
        Vehicle
        MapConfig
        Planner
        Scenario
        Geometry
    end

    methods (TestClassSetup)
        function loadDay7Configuration(testCase)
            testCase.ProjectRoot = fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(testCase.ProjectRoot,'matlab','single_vehicle'));
            testCase.Vehicle = jsondecode(fileread(fullfile(testCase.ProjectRoot, ...
                'config','vehicle_config_day7.json')));
            testCase.MapConfig = jsondecode(fileread(fullfile(testCase.ProjectRoot, ...
                'config','map_config_day7.json')));
            testCase.Planner = jsondecode(fileread(fullfile(testCase.ProjectRoot, ...
                'config','planner_config_day7.json')));
            testCase.Scenario = jsondecode(fileread(fullfile(testCase.ProjectRoot, ...
                'config','scenario_day7_four_vehicle.json')));
            testCase.Geometry = summon_map(testCase.MapConfig);
        end
    end

    methods (Test)
        function configuredDay7ScenarioIsLoaded(testCase)
            testCase.verifyEqual({testCase.Scenario.vehicles.id}, ...
                {'vehicle_001','vehicle_002','vehicle_003','vehicle_004'});
            testCase.verifyEqual([testCase.Scenario.vehicles.priority],[1,2,3,4]);
            starts = zeros(4,3);
            goals = zeros(4,3);
            for k = 1:4
                starts(k,:) = [testCase.Scenario.vehicles(k).start_pose.x_m, ...
                    testCase.Scenario.vehicles(k).start_pose.y_m, ...
                    testCase.Scenario.vehicles(k).start_pose.yaw_rad];
                goals(k,:) = [testCase.Scenario.vehicles(k).summon_goal.x_m, ...
                    testCase.Scenario.vehicles(k).summon_goal.y_m, ...
                    testCase.Scenario.vehicles(k).summon_goal.yaw_rad];
            end
            testCase.verifyEqual(starts,[4,25,-1.57;8,26,0;27,25,3.14;23,20,-1.57], ...
                'AbsTol',1e-12);
            testCase.verifyEqual(goals,[25,5,-1.57;20,6,-1.57;6,4,3.14;28,8,-1.57], ...
                'AbsTol',1e-12);
        end

        function mapAndTimeGridAreDay7Sized(testCase)
            vertices = testCase.MapConfig.boundary.vertices_xy;
            testCase.verifyEqual([min(vertices(:,1)),max(vertices(:,1)), ...
                min(vertices(:,2)),max(vertices(:,2))],[0,30,0,30], 'AbsTol',1e-12);
            st = testCase.Planner.st_occupancy;
            testCase.verifyEqual([st.x_min,st.x_max,st.y_min,st.y_max],[0,30,0,30], ...
                'AbsTol',1e-12);
            testCase.verifyEqual(st.dt,testCase.Planner.vhybrid.time_step_s, ...
                'AbsTol',1e-12);
        end

        function allConfiguredPosesFitBoundary(testCase)
            validate_st_occupancy_config(testCase.Planner);
            for k = 1:numel(testCase.Scenario.vehicles)
                vehicle = testCase.Scenario.vehicles(k);
                poses = [vehicle.start_pose.x_m,vehicle.start_pose.y_m,vehicle.start_pose.yaw_rad; ...
                    vehicle.summon_goal.x_m,vehicle.summon_goal.y_m,vehicle.summon_goal.yaw_rad];
                for p = 1:2
                    [collision,detail] = day6_static_collision_check(poses(p,:), ...
                        testCase.Geometry,testCase.Vehicle);
                    testCase.verifyFalse(collision, ...
                        sprintf('%s pose %d is outside the Day7 boundary (%s).', ...
                        vehicle.id,p,detail.type));
                end
            end
        end

        function initialVehicleSafetyRectanglesDoNotOverlap(testCase)
            planner = testCase.Planner;
            for i = 1:3
                for j = i+1:4
                    a = testCase.Scenario.vehicles(i).start_pose;
                    b = testCase.Scenario.vehicles(j).start_pose;
                    stateA = [a.x_m,a.y_m,a.yaw_rad,0,0];
                    stateB = [b.x_m,b.y_m,b.yaw_rad,0,0];
                    [collision,detail] = check_vehicle_vehicle_collision( ...
                        stateA,stateB,testCase.Vehicle,testCase.Vehicle,planner.day6);
                    testCase.verifyFalse(collision, ...
                        sprintf('Initial safety rectangles overlap: %s/%s.', ...
                        testCase.Scenario.vehicles(i).id,testCase.Scenario.vehicles(j).id));
                    testCase.verifyGreaterThan(detail.minimum_distance_m,0);
                end
            end
        end
    end
end
