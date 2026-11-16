classdef test_four_vehicle_uniform < matlab.unittest.TestCase
    %TEST_FOUR_VEHICLE_UNIFORM Public policy validation and fresh searches.
    properties
        Scenario
        Planner
        Task
    end
    methods (TestClassSetup)
        function setup(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
            testCase.Scenario = jsondecode(fileread(fullfile(root,'config', ...
                'scenario_four_vehicle_uniform_current.json')));
            testCase.Planner = jsondecode(fileread(fullfile(root,'config', ...
                'planner_config_four_vehicle_uniform.json')));
            testCase.Task = jsondecode(fileread(fullfile(root,'config', ...
                'four_vehicle_lateral_uniform.json')));
        end
    end
    methods (Test)
        function arbitraryLabelsAndChangedGoalAreAccepted(testCase)
            s = testCase.Scenario;
            s.vehicles(1).id = 'arbitrary_car';
            s.planning_order{1} = 'arbitrary_car';
            s.vehicles(4).summon_goal.x_m = 12;
            actual = day7_uniform_configuration(s,testCase.Planner);
            testCase.verifyEqual(actual.vehicles(4).summon_goal.x_m,12);
            testCase.verifyEqual(actual.planning_order{1},'arbitrary_car');
        end
        function scenarioOrderOverridesLegacyPriorityMetadata(testCase)
            s = testCase.Scenario;
            s.planning_order = flip(s.planning_order);
            actual = day7_uniform_configuration(s,testCase.Planner);
            testCase.verifyEqual(actual.vehicles(3).priority,1);
            testCase.verifyEqual(actual.vehicles(1).priority,4);
        end
        function arbitraryFiniteHeadingIsNormalized(testCase)
            s = testCase.Scenario;
            s.vehicles(2).start_pose.yaw_rad = 10*pi+0.2;
            actual = day7_uniform_configuration(s,testCase.Planner);
            testCase.verifyEqual(actual.vehicles(2).start_pose.yaw_rad,0.2,'AbsTol',1e-12);
        end
        function duplicateOrderIsRejected(testCase)
            s = testCase.Scenario; s.planning_order{4} = s.planning_order{1};
            testCase.verifyError(@()day7_uniform_configuration(s,testCase.Planner), ...
                'Day7Uniform:InvalidOrder');
        end
        function nonfinitePoseIsRejected(testCase)
            s = testCase.Scenario; s.vehicles(1).start_pose.x_m = NaN;
            testCase.verifyError(@()day7_uniform_configuration(s,testCase.Planner), ...
                'Day7Uniform:InvalidPose');
        end
        function inheritedVehicleOverridesAreRejected(testCase)
            p = testCase.Planner; p.day7_vehicle_search_overrides = struct();
            testCase.verifyError(@()day7_uniform_configuration(testCase.Scenario,p), ...
                'Day7Uniform:OverridesForbidden');
        end
        function historicalTrajectoryInputIsRejected(testCase)
            s = testCase.Scenario; s.initial_trajectory = struct();
            testCase.verifyError(@()day7_uniform_configuration(s,testCase.Planner), ...
                'Day7Uniform:OverridesForbidden');
        end
        function taskVehicleOverridesAreRejected(testCase)
            t = testCase.Task; t.vehicle_search_overrides = struct();
            testCase.verifyError(@()run_four_vehicle_lateral_planning(t,false), ...
                'Day7Uniform:TaskOverrides');
        end
        function disabledLateralLimitIsRejected(testCase)
            p = testCase.Planner; p.dynamics.lateral_acceleration_constraint_enabled = false;
            testCase.verifyError(@()day7_uniform_configuration(testCase.Scenario,p), ...
                'Day7Uniform:InvalidLateralLimit');
        end
        function invalidDynamicPruningIsRejected(testCase)
            p = testCase.Planner; p.vhybrid.dynamic_braking_dominance_pruning_enabled = 1;
            testCase.verifyError(@()day7_uniform_configuration(testCase.Scenario,p), ...
                'Day7Uniform:InvalidPruning');
        end
        function overlappingTerminalParkingIsRejectedBeforeSearch(testCase)
            t = testCase.Task; t.scenario = testCase.Scenario;
            t.scenario.vehicles(2).summon_goal = t.scenario.vehicles(1).summon_goal;
            actual = run_four_vehicle_lateral_planning(t,false);
            testCase.verifyFalse(actual.valid);
            testCase.verifyEqual(actual.search_time_s,0);
            testCase.verifyTrue(actual.pose_validation.goal_pairs(1).overlap);
        end
        function outOfBoundsPoseIsRejectedBeforeSearch(testCase)
            t = testCase.Task; t.scenario = testCase.Scenario;
            t.scenario.vehicles(1).start_pose.x_m = -1;
            actual = run_four_vehicle_lateral_planning(t,false);
            testCase.verifyFalse(actual.valid);
            testCase.verifyEqual(actual.search_time_s,0);
        end
        function renamingCarsPreservesFreshPathsAndCommonPolicy(testCase)
            t = testCase.Task; t.scenario = testCase.parallelScene();
            first = run_four_vehicle_lateral_planning(t,false);
            t.scenario = testCase.parallelScene({'zeta','alpha','delta','beta'});
            renamed = run_four_vehicle_lateral_planning(t,false);
            testCase.assertTrue(first.valid);
            testCase.assertTrue(renamed.valid);
            a = first.stages(end); b = renamed.stages(end);
            testCase.verifyTrue(testCase.samePaths(a.paths,b.paths));
            testCase.verifyEqual({a.per_vehicle.effective_search_config}, ...
                repmat({first.effective_configuration.common_search_config},1,4));
            testCase.verifyEqual({b.per_vehicle.effective_search_config}, ...
                {a.per_vehicle.effective_search_config});
            testCase.verifyTrue(all([first.pairwise_validation.valid]));
            testCase.verifyTrue(all([renamed.pairwise_validation.valid]));
        end
    end
    methods (Static, Access=private)
        function s = parallelScene(ids)
            if nargin<1, ids = {'car_a','car_b','car_c','car_d'}; end
            vehicles = repmat(struct('id','','start_pose',struct(), ...
                'summon_goal',struct()),1,4);
            for k=1:4
                vehicles(k).id = ids{k};
                vehicles(k).start_pose = struct('x_m',5,'y_m',4+7*(k-1),'yaw_rad',0);
                vehicles(k).summon_goal = struct('x_m',10,'y_m',4+7*(k-1),'yaw_rad',0);
            end
            s = struct('vehicles',vehicles,'planning_order',{ids});
        end
        function tf = samePaths(a,b)
            tf = true;
            fields = {'x','y','theta','v','t','acceleration','steering_angle','gear'};
            for k=1:numel(a)
                for j=1:numel(fields)
                    tf = tf && isequal(a{k}.(fields{j}),b{k}.(fields{j}));
                end
            end
        end
    end
end
