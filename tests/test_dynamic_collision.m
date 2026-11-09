classdef test_dynamic_collision < matlab.unittest.TestCase
    %TEST_DYNAMIC_COLLISION Day6 顺序规划、动态矩形和连续段检查验收。
    % 输入：各用例构造的可复现轨迹、车辆和集中配置；不调用控制器。
    % 输出：matlab.unittest.TestResult，包含逐项通过/失败和诊断信息。
    % 逻辑：仅调用公开接口。矩形为最终判据，资源表为保守约束；
    % 追尾、交叉、时间错开、等待、扫掠及减速剪枝分别独立验证。

    properties (TestParameter)
        accelerationCase = struct( ...
            'accelerating', struct('a', 2, 'v_next', 2, 'distance', 0.75), ...
            'constantSpeed', struct('a', 0, 'v_next', 1, 'distance', 0.5), ...
            'decelerating', struct('a', -2, 'v_next', 0, 'distance', 0.25));
    end

    methods (TestClassSetup)
        function addDay6Functions(testCase)
            % 路径由 fixture 管理，结束后自动恢复；测试不写任何输出文件。
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, 'matlab', 'single_vehicle')));
        end
    end

    methods (Test)
        function obstacleStoresTimeStatesAndOwner(testCase)
            planner = testCase.smallPlanner();
            trajectory = [5, 10, 0, 1, 0; 6, 10, 0, 0, 1];
            obstacle = dynamic_vehicle_obstacle(trajectory, ...
                testCase.smallVehicle(), 'vehicle_002', planner);
            testCase.verifyEqual(obstacle.id, 'vehicle_002');
            testCase.verifyEqual(obstacle.states, trajectory, 'AbsTol', 1e-12);
            testCase.verifyTrue(obstacle.hold_end);
            testCase.verifyGreaterThan(obstacle.footprint.length, 0);
        end

        function predictionInterpolatesPositionVelocityAndTime(testCase)
            planner = testCase.smallPlanner();
            obstacle = dynamic_vehicle_obstacle([5, 10, 0, 2, 1; ...
                9, 10, 0, 0, 3], testCase.smallVehicle(), 'vehicle_002', planner);
            [state, active] = predict_vehicle_pose(obstacle, 2);
            testCase.verifyTrue(active);
            testCase.verifyEqual(state, [7, 10, 0, 1, 2], 'AbsTol', 1e-12);
        end

        function vectorPredictionPreservesRequestedTimes(testCase)
            obstacle = dynamic_vehicle_obstacle([5, 10, 0, 2, 1; ...
                9, 10, 0, 0, 3], testCase.smallVehicle(), 'vehicle_002', ...
                testCase.smallPlanner());
            [states, active] = predict_vehicle_pose(obstacle, [1, 2, 3, 5]);
            testCase.verifySize(states, [4, 5]);
            testCase.verifyEqual(active(:), true(4, 1));
            testCase.verifyEqual(states(:, 5), [1; 2; 3; 5], 'AbsTol', 1e-12);
            testCase.verifyEqual(states(4, 1:4), [9, 10, 0, 0], 'AbsTol', 1e-12);
        end

        function predictionDoesNotExtrapolateBeforeTrajectory(testCase)
            obstacle = dynamic_vehicle_obstacle([5, 10, 0, 0, 1; ...
                5, 10, 0, 0, 3], testCase.smallVehicle(), 'vehicle_002', ...
                testCase.smallPlanner());
            [~, active] = predict_vehicle_pose(obstacle, 0.5);
            testCase.verifyFalse(active);
        end

        function releasedTrajectoryBecomesInactiveAfterEnd(testCase)
            planner = testCase.smallPlanner();
            planner.day6.goal_hold_enabled = false;
            planner.st_occupancy.trajectory_end_policy = 'release';
            obstacle = dynamic_vehicle_obstacle([5, 10, 0, 0, 0; ...
                5, 10, 0, 0, 1], testCase.smallVehicle(), 'vehicle_002', planner);
            [~, active] = predict_vehicle_pose(obstacle, 2);
            testCase.verifyFalse(obstacle.hold_end);
            testCase.verifyFalse(active);
        end

        function yawInterpolationTakesShortestWrappedRoute(testCase)
            obstacle = dynamic_vehicle_obstacle([5, 10, 179*pi/180, 0, 0; ...
                5, 10, -179*pi/180, 0, 1], testCase.smallVehicle(), ...
                'vehicle_002', testCase.smallPlanner());
            [state, active] = predict_vehicle_pose(obstacle, 0.5);
            testCase.verifyTrue(active);
            testCase.verifyEqual(abs(state(3)), pi, 'AbsTol', 1e-12);
        end

        function invalidTrajectoryTimesAreRejected(testCase)
            trajectory = [5, 10, 0, 0, 1; 6, 10, 0, 0, 1];
            testCase.verifyError(@() dynamic_vehicle_obstacle(trajectory, ...
                testCase.smallVehicle(), 'vehicle_002', testCase.smallPlanner()), ...
                'DynamicCollision:InvalidTrajectory');
        end

        function negativeSpeedIsRejected(testCase)
            testCase.verifyError(@() dynamic_vehicle_obstacle( ...
                [5, 10, 0, -1, 0], testCase.smallVehicle(), 'vehicle_002', ...
                testCase.smallPlanner()), 'DynamicCollision:InvalidTrajectory');
        end

        function invalidPredictionTimeIsRejected(testCase)
            obstacle = dynamic_vehicle_obstacle([5, 10, 0, 0, 0], ...
                testCase.smallVehicle(), 'vehicle_002', testCase.smallPlanner());
            testCase.verifyError(@() predict_vehicle_pose(obstacle, NaN), ...
                'DynamicCollision:InvalidTime');
        end

        function rearEndRectanglesCollide(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 1, 0], [11, 10, 0, 0, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(detail.rectangle_overlap);
            testCase.verifyEqual(detail.minimum_distance_m, 0, 'AbsTol', 1e-12);
        end

        function crossingRectanglesCollide(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 1, 0], [10, 10, pi/2, 1, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(detail.rectangle_overlap);
        end

        function rawGeometricClearanceUsesVehicleEdges(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 0, 0], [10, 11, 0, 0, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyFalse(collision);
            testCase.verifyEqual(detail.minimum_distance_m, 0.6, 'AbsTol', 1e-12);
            testCase.verifyEqual(detail.inflated_distance_m, 0.6, 'AbsTol', 1e-12);
        end

        function safetyMarginCanRejectRawSeparatedRectangles(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            vehicle.collision_margin_m.side = 0.31;
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 0, 0], [10, 11, 0, 0, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyTrue(collision);
            testCase.verifyEqual(detail.minimum_distance_m, 0.6, 'AbsTol', 1e-12);
            testCase.verifyEqual(detail.inflated_distance_m, 0, 'AbsTol', 1e-12);
        end

        function configuredMinimumSafetyDistanceIsEnforced(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            planner.day6.minimum_safety_distance_m = 0.7;
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 0, 0], [10, 11, 0, 0, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyTrue(collision);
            testCase.verifyEqual(detail.minimum_distance_m, 0.6, 'AbsTol', 1e-12);
        end

        function circlesAndAabbDoNotReplaceRotatedSat(testCase)
            % 两个斜向平行矩形侧向分离；圆和世界轴包围盒仍然重叠。
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            other = [10-0.6*sin(pi/4), 10+0.6*cos(pi/4), pi/4, 0, 0];
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, pi/4, 0, 0], other, vehicle, vehicle, planner.day6);
            testCase.verifyTrue(detail.circle_overlap);
            testCase.verifyTrue(detail.aabb_overlap);
            testCase.verifyFalse(detail.rectangle_overlap);
            testCase.verifyFalse(collision);
            testCase.verifyEqual(detail.minimum_distance_m, 0.2, 'AbsTol', 1e-12);
        end

        function headingChangesFinalRectangleDecision(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            [parallelCollision, ~] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 0, 0], [11.4, 10, 0, 0, 0], ...
                vehicle, vehicle, planner.day6);
            [rotatedCollision, ~] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 0, 0], [11.4, 10, pi/2, 0, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyTrue(parallelCollision);
            testCase.verifyFalse(rotatedCollision);
        end

        function rectangleContactIsConservativelyCollision(testCase)
            vehicle = testCase.smallVehicle();
            planner = testCase.smallPlanner();
            [collision, detail] = check_vehicle_vehicle_collision( ...
                [10, 10, 0, 0, 0], [11.5, 10, 0, 0, 0], ...
                vehicle, vehicle, planner.day6);
            testCase.verifyTrue(collision);
            testCase.verifyEqual(detail.minimum_distance_m, 0, 'AbsTol', 1e-12);
        end

        function rearEndTrajectoryReportsTimePositionAndOwner(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            ego = [3, 10, 0, 2, 0; 13, 10, 0, 2, 5];
            obstacle = dynamic_vehicle_obstacle([10, 10, 0, 0, 0; ...
                10, 10, 0, 0, 5], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.physical_collision);
            testCase.verifyEqual(report.obstacle_id, 'vehicle_002');
            testCase.verifyNotEmpty(report.reason);
            testCase.verifyGreaterThan(report.conflict_time_s, 0);
            testCase.verifyLessThan(report.conflict_time_s, 5);
            testCase.verifySize(report.conflict_position_xy, [1, 2]);
            testCase.verifyTrue(all(isfinite(report.conflict_position_xy)));
        end

        function crossingTrajectoriesFindIntermediateConflict(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            ego = [4, 10, 0, 3, 0; 16, 10, 0, 3, 4];
            obstacle = dynamic_vehicle_obstacle([10, 4, pi/2, 3, 0; ...
                10, 16, pi/2, 0, 4], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.has_intermediate_conflict);
            testCase.verifyGreaterThan(report.checked_samples, 2);
            testCase.verifyGreaterThan(report.conflict_time_s, 0);
            testCase.verifyLessThan(report.conflict_time_s, 4);
        end

        function sameSpaceAtDifferentTimesIsSafe(testCase)
            planner = testCase.smallPlanner();
            planner.day6.goal_hold_enabled = false;
            planner.st_occupancy.trajectory_end_policy = 'release';
            vehicle = testCase.smallVehicle();
            ego = [4, 10, 0, 3, 0; 16, 10, 0, 3, 4];
            obstacle = dynamic_vehicle_obstacle([10, 4, pi/2, 3, 5; ...
                10, 16, pi/2, 0, 9], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyFalse(collision);
            testCase.verifyFalse(report.physical_collision);
            testCase.verifyFalse(report.resource_conflict);
        end

        function timeSeparatedReservationsUseDifferentResourceLayers(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            high = [10, 4, pi/2, 3, 5; 10, 16, pi/2, 0, 9];
            ego = [4, 10, 0, 3, 0; 16, 10, 0, 3, 4];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner), high, vehicle, 'vehicle_002');
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner, map);
            testCase.verifyFalse(collision);
            testCase.verifyFalse(report.physical_collision);
            testCase.verifyFalse(report.resource_conflict);
        end

        function waitingAwayFromCrossingIsSafe(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            ego = [7, 10, 0, 0, 0; 7, 10, 0, 0, 4];
            obstacle = dynamic_vehicle_obstacle([10, 4, pi/2, 3, 0; ...
                10, 16, pi/2, 0, 4], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyFalse(collision);
            testCase.verifyGreaterThan(report.minimum_distance_m, 0);
        end

        function waitingInsideCrossingIsRejected(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            ego = [10, 10, 0, 0, 0; 10, 10, 0, 0, 4];
            obstacle = dynamic_vehicle_obstacle([10, 4, pi/2, 3, 0; ...
                10, 16, pi/2, 0, 4], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.physical_collision);
            testCase.verifyTrue(report.has_intermediate_conflict);
        end

        function parkedGoalContinuesToBlockLaterVehicles(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            obstacle = dynamic_vehicle_obstacle([10, 10, 0, 0, 0; ...
                10, 10, 0, 0, 1], vehicle, 'vehicle_002', planner);
            ego = [10, 10, 0, 0, 5; 10, 10, 0, 0, 6];
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.physical_collision);
            testCase.verifyEqual(report.obstacle_id, 'vehicle_002');
        end

        function parallelTrajectoriesReportConservativeDistanceBound(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            ego = [3, 10, 0, 2, 0; 13, 10, 0, 2, 5];
            obstacle = dynamic_vehicle_obstacle([3, 11, 0, 2, 0; ...
                13, 11, 0, 0, 5], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyFalse(collision);
            testCase.verifyEqual(report.minimum_distance_m, 0.6, 'AbsTol', 1e-10);
            testCase.verifyGreaterThanOrEqual(report.minimum_distance_lower_bound_m, 0);
            testCase.verifyLessThanOrEqual(report.minimum_distance_lower_bound_m, ...
                report.minimum_distance_m + 1e-12);
        end

        function rapidCrossingCannotTunnelBetweenEndpoints(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.tinyVehicle();
            ego = [5, 10, 0, 10, 0; 15, 10, 0, 10, 1];
            obstacle = dynamic_vehicle_obstacle([10, 10, 0, 0, 0; ...
                10, 10, 0, 0, 1], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.has_intermediate_conflict);
            testCase.verifyGreaterThan(report.conflict_time_s, 0);
            testCase.verifyLessThan(report.conflict_time_s, 1);
        end

        function coarseResourceConflictIsRejectedEvenWithoutPhysicalOverlap(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.tinyVehicle();
            high = [10.35, 10.35, 0, 0, 0; 10.35, 10.35, 0, 0, 1];
            ego = [10.15, 10.15, 0, 0, 0; 10.15, 10.15, 0, 0, 1];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner), ...
                high, vehicle, 'vehicle_002');
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner, map);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyFalse(report.physical_collision);
            testCase.verifyEqual(report.obstacle_id, 'vehicle_002');
        end

        function disjointResourceReservationsRemainSafe(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            high = [3, 12, 0, 2, 0; 13, 12, 0, 0, 5];
            ego = [3, 10, 0, 2, 0; 13, 10, 0, 2, 5];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner), high, vehicle, 'vehicle_002');
            [collision, report] = check_trajectory_conflict(ego, obstacle, vehicle, planner, map);
            testCase.verifyFalse(collision);
            testCase.verifyFalse(report.resource_conflict);
            testCase.verifyFalse(report.physical_collision);
        end

        function distantCachedEnvelopeMatchesFullResourceQuery(testCase)
            % 将包络改成无限大仅强制公开接口执行完整栅格查询，比较安全结论。
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            high = [20, 15, 0, 0, 0; 20, 15, 0, 0, 1];
            ego = [5, 5, 0, 1, 0; 6, 5, 0, 1, 1];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            cached = prepare_dynamic_context(mark_trajectory_occupancy( ...
                st_occupancy_map(planner), high, vehicle, 'vehicle_002'), obstacle);
            full = cached;
            full.dynamic_layer_bounds = repmat([-Inf, Inf, -Inf, Inf], full.nt, 1);
            options = struct('collect_distance_samples', false);
            [cachedHit, cachedReport] = check_trajectory_conflict( ...
                ego, obstacle, vehicle, planner, cached, options);
            [fullHit, fullReport] = check_trajectory_conflict( ...
                ego, obstacle, vehicle, planner, full, options);
            testCase.verifyEqual(cachedHit, fullHit);
            testCase.verifyFalse(cachedHit);
            testCase.verifyEqual(cachedReport.resource_indices, fullReport.resource_indices);
            testCase.verifyEqual(cachedReport.resources_checked, 0);
            testCase.verifyGreaterThan(fullReport.resources_checked, 0);
        end

        function nearbyCachedEnvelopePreservesAllConflictingResources(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            high = [10, 10, 0, 0, 0; 10, 10, 0, 0, 1];
            ego = [10, 10, 0, 0, 0; 10, 10, 0, 0, 1];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            cached = prepare_dynamic_context(mark_trajectory_occupancy( ...
                st_occupancy_map(planner), high, vehicle, 'vehicle_002'), obstacle);
            full = cached;
            full.dynamic_layer_bounds = repmat([-Inf, Inf, -Inf, Inf], full.nt, 1);
            options = struct('collect_distance_samples', false);
            [cachedHit, cachedReport] = check_trajectory_conflict( ...
                ego, obstacle, vehicle, planner, cached, options);
            [fullHit, fullReport] = check_trajectory_conflict( ...
                ego, obstacle, vehicle, planner, full, options);
            testCase.verifyEqual(cachedHit, fullHit);
            testCase.verifyTrue(cachedHit);
            testCase.verifyTrue(cachedReport.resource_conflict);
            testCase.verifyEqual(cachedReport.resource_indices, fullReport.resource_indices);
            testCase.verifyGreaterThan(cachedReport.resources_checked, 0);
            testCase.verifyEqual(cachedReport.obstacle_id, 'vehicle_002');
        end

        function highPriorityGoalResourcePersistsToHorizon(testCase)
            planner = testCase.smallPlanner();
            high = [10, 10, 0, 0, 0; 10, 10, 0, 0, 1];
            map = mark_trajectory_occupancy(st_occupancy_map(planner), ...
                high, testCase.smallVehicle(), 'vehicle_002');
            [occupied, type, owner] = query_occupancy(10, 10, 8, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
            testCase.verifyEqual(owner, 'vehicle_002');
        end

        function spatialOutOfBoundsIsRejectedWithResourceContext(testCase)
            planner = testCase.smallPlanner();
            map = st_occupancy_map(planner);
            [collision, report] = check_trajectory_conflict( ...
                [30, 10, 0, 0, 0; 30, 10, 0, 0, 1], [], ...
                testCase.smallVehicle(), planner, map);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyNotEmpty(report.reason);
        end

        function insideRearAxleDoesNotHideOutsideVehicleFootprint(testCase)
            % 动态车为空时也不能让包络预筛选跳过车身范围检查。
            planner = testCase.smallPlanner();
            map = st_occupancy_map(planner);
            [collision, report] = check_trajectory_conflict( ...
                [0.1, 10, 0, 0, 0; 0.1, 10, 0, 0, 1], [], ...
                testCase.smallVehicle(), planner, map, ...
                struct('collect_distance_samples', false));
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyTrue(report.collision);
            testCase.verifyFalse(report.physical_collision);
            testCase.verifyNotEmpty(report.reason);
        end

        function timeHorizonIsRejectedWithResourceContext(testCase)
            planner = testCase.smallPlanner();
            map = st_occupancy_map(planner);
            [collision, report] = check_trajectory_conflict( ...
                [10, 10, 0, 0, 9.5; 10, 10, 0, 0, 10], [], ...
                testCase.smallVehicle(), planner, map);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyNotEmpty(report.reason);
        end

        function emptyObstacleSetIsFree(testCase)
            [collision, report] = check_trajectory_conflict( ...
                [3, 10, 0, 1, 0; 4, 10, 0, 1, 1], [], ...
                testCase.smallVehicle(), testCase.smallPlanner());
            testCase.verifyFalse(collision);
            testCase.verifyFalse(report.physical_collision);
            testCase.verifyFalse(report.resource_conflict);
            testCase.verifyEqual(report.minimum_distance_m, Inf);
        end

        function motionSamplesReplayBicycleModel(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            start = [10, 10, 0.3, 1, 0];
            steering = 0.2;
            acceleration = 1;
            next = testCase.integrate(start, steering, acceleration, 0.5, vehicle, planner);
            motion = day6_motion_trajectory(start, next, steering, acceleration, vehicle, planner);
            states = testCase.motionStates(motion);
            middleTime = states(2, 5) - start(5);
            expected = testCase.integrate(start, steering, acceleration, middleTime, vehicle, planner);
            testCase.verifyGreaterThan(size(states, 1), 2);
            testCase.verifyEqual(states(1, :), start, 'AbsTol', 1e-12);
            testCase.verifyEqual(states(end, :), next, 'AbsTol', 1e-12);
            testCase.verifyEqual(states(2, :), expected, 'AbsTol', 1e-12);
        end

        function zeroSpeedZeroAccelerationProducesWaitingMotion(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            start = [10, 10, 0.3, 0, 1];
            next = [10, 10, 0.3, 0, 1.5];
            motion = day6_motion_trajectory(start, next, 0, 0, vehicle, planner);
            states = testCase.motionStates(motion);
            testCase.verifyEqual(states(:, 1:4), ...
                repmat(start(1:4), size(states, 1), 1), 'AbsTol', 1e-12);
            testCase.verifyTrue(all(diff(states(:, 5)) > 0));
        end

        function accelerationCruiseAndBrakingEdgesAreChecked(testCase, accelerationCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            start = [10, 10, 0, 1, 0];
            next = testCase.integrate(start, 0, accelerationCase.a, 0.5, vehicle, planner);
            motion = day6_motion_trajectory(start, next, 0, accelerationCase.a, vehicle, planner);
            states = testCase.motionStates(motion);
            obstacle = dynamic_vehicle_obstacle([20, 15, 0, 0, 0; ...
                20, 15, 0, 0, 1], vehicle, 'vehicle_002', planner);
            [collision, report] = check_trajectory_conflict(motion, obstacle, vehicle, planner);
            testCase.verifyFalse(collision);
            testCase.verifyEqual(states(end, 4), accelerationCase.v_next, 'AbsTol', 1e-12);
            testCase.verifyEqual(states(end, 1)-start(1), accelerationCase.distance, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(report.checked_samples, 2);
        end

        function brakingMiddleCollisionStopsFasterSameSteeringControls(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.control_acceleration_samples_mps2 = [-2, 0, 2];
            planner.vhybrid.control_steering_samples_rad = 0;
            vehicle = testCase.tinyVehicle();
            parent = vhybrid_node([10, 10, 0, 2, 0], 0, 0, 5, 0, 0, [20, 20, 0, 8, 0], 6);
            obstacle = dynamic_vehicle_obstacle([10.4, 10, 0, 0, 0; ...
                10.4, 10, 0, 0, 1], vehicle, 'vehicle_002', planner);
            [children, statistics] = vhybrid_expand_node(parent, ...
                testCase.openGeometry(), vehicle, planner, [20, 10, 0, 0, 0], [], obstacle);
            testCase.verifyEmpty(children);
            testCase.verifyEqual(statistics.braking_midpoint_pruned, 1);
            testCase.verifyEqual(statistics.faster_controls_skipped, 2);
            testCase.verifyGreaterThanOrEqual(statistics.dynamic_pruned, 1);
            testCase.verifyNotEmpty(fieldnames(statistics.reason_counts));
        end

        function openStraightExpansionKeepsAccelerationCruiseAndBraking(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.control_acceleration_samples_mps2 = [-2, 0, 2];
            planner.vhybrid.control_steering_samples_rad = 0;
            vehicle = testCase.smallVehicle();
            parent = vhybrid_node([10, 10, 0, 2, 0], 0, 0, 5, 0, 0, [20, 20, 0, 8, 0], 6);
            [children, statistics] = vhybrid_expand_node(parent, ...
                testCase.openGeometry(), vehicle, planner, [20, 10, 0, 0, 0], ...
                st_occupancy_map(planner), []);
            testCase.verifyEqual(sort([children.v]), [1, 2, 3], 'AbsTol', 1e-12);
            testCase.verifyEqual(statistics.valid, 3);
            testCase.verifyEqual(statistics.collision_pruned, 0);
        end

        function expansionStillChecksEndpointAfterStateReuse(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.reverse_enabled = false;
            planner.vhybrid.control_acceleration_samples_mps2 = 0;
            planner.vhybrid.control_steering_samples_rad = 0;
            vehicle = testCase.tinyVehicle();
            geometry = testCase.openGeometry();
            geometry.boundary_xy = [0, 0; 11.05, 0; 11.05, 20; 0, 20; 0, 0];
            geometry.bounds = [0, 0, 11.05, 20];
            parent = vhybrid_node([10, 10, 0, 2, 0], 0, 0, 5, 0, 0, ...
                [20, 20, 0, 8, 0], 6);

            [children, statistics] = vhybrid_expand_node(parent,geometry, ...
                vehicle,planner,[20, 10, 0, 0, 0]);

            testCase.verifyEmpty(children);
            testCase.verifyEqual(statistics.collision_pruned,1);
            testCase.verifyEqual(statistics.reason_counts.static_collision,1);
        end

        function simultaneousStartRequiresPositiveAcceleration(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.control_acceleration_samples_mps2 = [-2, 0, 2];
            planner.vhybrid.control_steering_samples_rad = 0;
            vehicle = testCase.smallVehicle();
            parent = vhybrid_node([10, 10, 0, 0, 0], 0, 0, 0, 0, 0, [20, 20, 0, 0, 0], 1);
            [children, statistics] = vhybrid_expand_node(parent, ...
                testCase.openGeometry(), vehicle, planner, [20, 10, 0, 0, 0], ...
                st_occupancy_map(planner), []);
            testCase.verifyEqual(numel(children), 1);
            testCase.verifyGreaterThan(children(1).v, 0);
            testCase.verifyGreaterThan(children(1).x, parent.x);
            testCase.verifyEqual(statistics.constraint_pruned, 2);
        end

        function staticObstacleEdgeCrossingCannotEscapeCornerOnlyChecks(testCase)
            % 车身与窄条十字相交：双方顶点均不在对方内部，边相交仍必须拒绝。
            geometry = testCase.openGeometry();
            geometry.obstacles = {[10.2, 9; 10.3, 9; 10.3, 11; 10.2, 11; 10.2, 9]};
            [collision, detail] = day6_static_collision_check( ...
                [10, 10, 0], geometry, testCase.smallVehicle());
            testCase.verifyTrue(collision);
            testCase.verifyNotEmpty(detail);
        end

        function staticObstacleContainedInsideVehicleIsRejected(testCase)
            geometry = testCase.openGeometry();
            geometry.obstacles = {[10.2, 9.95; 10.3, 9.95; ...
                10.3, 10.05; 10.2, 10.05; 10.2, 9.95]};
            [collision, detail] = day6_static_collision_check( ...
                [10, 10, 0], geometry, testCase.smallVehicle());
            testCase.verifyTrue(collision);
            testCase.verifyNotEmpty(detail);
        end

        function staticBoundaryChecksEntireVehicleFootprint(testCase)
            [collision, detail] = day6_static_collision_check( ...
                [0.1, 10, 0], testCase.openGeometry(), testCase.smallVehicle());
            testCase.verifyTrue(collision);
            testCase.verifyNotEmpty(detail);
        end

        function publicSearcherStartsAtRestAndStopsAtGoal(testCase)
            % 集成验收由真正的搜索器生成轨迹，不向搜索器输入预设路径。
            planner = testCase.smallPlanner();
            planner.vhybrid.control_acceleration_samples_mps2 = [-2, 0, 2];
            planner.vhybrid.control_steering_samples_rad = 0;
            planner.vhybrid.max_search_nodes = 500;
            planner.vhybrid.max_search_time_s = 10;
            vehicle = testCase.smallVehicle();
            map = st_occupancy_map(planner);
            path = summon_vhybrid_astar([5, 5, 0, 0, 0], [10, 5, 0, 0, 0], ...
                testCase.openGeometry(), vehicle, planner, map, []);
            [collision, report] = check_trajectory_conflict(path, [], vehicle, planner, map);
            testCase.assertTrue(path.valid);
            testCase.verifyEqual([path.v(1), path.v(end)], [0, 0], 'AbsTol', 1e-10);
            testCase.verifyEqual(path.t(1), 0, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(path.v(2), 0);
            testCase.verifyGreaterThan(path.x(2), path.x(1));
            testCase.verifyEqual([path.x(end), path.y(end)], [10, 5], ...
                'AbsTol', planner.vhybrid.goal_position_tolerance_m);
            testCase.verifyFalse(collision);
            testCase.verifyFalse(report.resource_conflict);
        end

        function publicSearcherRejectsOccupiedStartWithErrorNine(testCase)
            planner = testCase.smallPlanner();
            vehicle = testCase.smallVehicle();
            high = [10, 10, 0, 0, 0; 10, 10, 0, 0, 1];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner), high, vehicle, 'vehicle_002');
            path = summon_vhybrid_astar([10, 10, 0, 0, 0], [20, 10, 0, 0, 0], ...
                testCase.openGeometry(), vehicle, planner, map, obstacle);
            testCase.verifyFalse(path.valid);
            testCase.verifyEqual(path.error_code, uint8(9));
            testCase.verifyTrue(path.dynamic_validation.resource_conflict);
            testCase.verifyEqual(path.dynamic_validation.obstacle_id, 'vehicle_002');
            testCase.verifyNotEmpty(path.failure_diagnostics.reason);
        end

        function publicSearcherDoesNotAcceptUnsafeGoalResidence(testCase)
            % 高优先级车未来到达目标并长期驻留；不要求求解器找到替代解。
            planner = testCase.smallPlanner();
            planner.vhybrid.control_acceleration_samples_mps2 = [-2, 0, 2];
            planner.vhybrid.control_steering_samples_rad = 0;
            planner.vhybrid.max_search_nodes = 20;
            planner.vhybrid.max_search_time_s = 2;
            vehicle = testCase.smallVehicle();
            high = [20, 5, pi, 10/7, 0; 10, 5, pi, 0, 7];
            obstacle = dynamic_vehicle_obstacle(high, vehicle, 'vehicle_002', planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner), high, vehicle, 'vehicle_002');
            path = summon_vhybrid_astar([5, 5, 0, 0, 0], [10, 5, 0, 0, 0], ...
                testCase.openGeometry(), vehicle, planner, map, obstacle);
            testCase.verifyFalse(path.valid);
            testCase.verifyNotEqual(path.error_code, uint8(0));
            testCase.verifyNotEmpty(path.failure_diagnostics.reason);
            testCase.verifyFalse(path.failure_diagnostics.automatic_entry_time_shift_applied);
        end

        function heuristicWeightChangesOnlyHeuristicAndKeepsNodeFConsistent(testCase)
            planner = testCase.smallPlanner();
            parent = vhybrid_node([5, 5, 0, 1, 0], 0, 0, 0, 3, 0, [10, 10, 0, 4, 0], 1);
            child = [5.5, 5, 0, 1, 0.5];
            goal = [10, 5, 0, 0, 0];
            planner.vhybrid.heuristic_weight = 1;
            [plainG, plainH, plainF] = vhybrid_cost(parent, child, goal, planner, 0.5, 0);
            planner.vhybrid.heuristic_weight = 2;
            [weightedG, weightedH, weightedF] = vhybrid_cost(parent, child, goal, planner, 0.5, 0);
            node = vhybrid_node(child, 0, 0, 1, weightedG, weightedH, [11, 10, 0, 4, 1], 2);
            testCase.verifyEqual(weightedG, plainG, 'AbsTol', 1e-12);
            testCase.verifyEqual(weightedH, 2*plainH, 'AbsTol', 1e-12);
            testCase.verifyEqual(plainF, plainG+plainH, 'AbsTol', 1e-12);
            testCase.verifyEqual(weightedF, weightedG+weightedH, 'AbsTol', 1e-12);
            testCase.verifyEqual(node.f_cost, weightedF, 'AbsTol', 1e-12);
        end

        function fastDubinsConnectionHasStrictTimesAndModelReplay(testCase)
            % 使用实际车型和公开解析连接器；检查高速相位尾部不能产生重复时间。
            planner = testCase.smallPlanner();
            planner.vhybrid.goal_connection_distance_m = 40;
            planner.vhybrid.goal_connection_speed_samples_mps = 5;
            planner.vhybrid.reference_speed_mps = 5;
            vehicle = testCase.actualVehicle();
            start = [4, 17, -pi/2, 0, 0];
            goal = [24, 15, pi/2, 0, 0];
            parent = vhybrid_node(start, 0, 0, 0, 0, 0, [8, 34, 0, 0, 0], 1);
            [connected, nodes, ~] = vhybrid_goal_connection(parent, goal, ...
                testCase.openGeometry(), vehicle, planner);
            testCase.assertTrue(connected);
            residuals = testCase.connectionReplayResiduals(start, nodes, vehicle, planner);
            testCase.verifyTrue(all(diff([start(5); vertcat(nodes.t)]) > 0));
            testCase.verifyGreaterThan(max(vertcat(nodes.v)), 4);
            testCase.verifyEqual(nodes(end).v, 0, 'AbsTol', 1e-9);
            testCase.verifyEqual([nodes(end).x, nodes(end).y], goal(1:2), ...
                'AbsTol', planner.vhybrid.goal_position_tolerance_m);
            testCase.verifyEqual(mod(nodes(end).theta-goal(3)+pi, 2*pi)-pi, ...
                0, 'AbsTol', planner.vhybrid.goal_heading_tolerance_rad);
            testCase.verifyLessThan(max(residuals(:)), 1e-8);
        end

        function fullTurnBoundaryCheckFindsUnsafeMiddleWithSafeEndpoints(testCase)
            % 完整右转一圈回到起点；端点安全不能证明圆弧中间车角安全。
            vehicle = testCase.actualVehicle();
            radius = vehicle.dimensions_m.wheelbase/tan(vehicle.steering.maximum_steer_rad);
            geometry = testCase.openGeometry();
            curve = struct('lengths', [2*pi*radius, 0, 0], 'types', [-1, 0, 0]);
            [startCollision, ~] = day6_static_collision_check([10, 10, 0], geometry, vehicle);
            [endCollision, ~] = day6_static_collision_check([10, 10, -2*pi], geometry, vehicle);
            [outside, detail] = vhybrid_curve_boundary_check([10, 10, 0], ...
                curve, radius, geometry, vehicle);
            testCase.verifyFalse(startCollision);
            testCase.verifyFalse(endCollision);
            testCase.verifyTrue(detail.evaluated);
            testCase.verifyTrue(outside);
            testCase.verifyLessThan(detail.swept_bounds(2), 0);
            testCase.verifyEqual(detail.first_rejected_segment, 1);
        end

        function straightCurveBoundaryCheckKeepsEntireVehicleInside(testCase)
            vehicle = testCase.actualVehicle();
            radius = vehicle.dimensions_m.wheelbase/tan(vehicle.steering.maximum_steer_rad);
            curve = struct('lengths', [5, 0, 0], 'types', [0, 0, 0]);
            [outside, detail] = vhybrid_curve_boundary_check([10, 10, 0], ...
                curve, radius, testCase.openGeometry(), vehicle);
            testCase.verifyTrue(detail.evaluated);
            testCase.verifyFalse(outside);
            testCase.verifyGreaterThanOrEqual(detail.swept_bounds(1:2), [0, 0]);
            testCase.verifyLessThanOrEqual(detail.swept_bounds(3:4), [30, 20]);
        end

        function dubinsHeuristicMatchesShortestCandidateAndWeightOnlyScalesH(testCase)
            vehicle = testCase.actualVehicle();
            planner = testCase.smallPlanner();
            cfg = planner.vhybrid;
            cfg.heuristic_method = 'dubins';
            cfg.minimum_turning_radius_m = vehicle.dimensions_m.wheelbase/ ...
                tan(vehicle.steering.maximum_steer_rad);
            cfg.heuristic_weight = 1;
            state = [24.642, 15.737, -0.684, 3, 11];
            goal = [24, 15, pi/2, 0, 0];
            candidates = vhybrid_dubins_candidates(state(1:3), goal(1:3), ...
                cfg.minimum_turning_radius_m);
            [plainH, plainDistance, method] = vhybrid_heuristic(state, goal, cfg);
            cfg.heuristic_weight = 2;
            [weightedH, weightedDistance, ~] = vhybrid_heuristic(state, goal, cfg);
            testCase.assertNotEmpty(candidates);
            testCase.verifyEqual(method, 'dubins');
            testCase.verifyEqual(plainDistance, sum(candidates(1).lengths), 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(plainDistance, 5*hypot(state(1)-goal(1), state(2)-goal(2)));
            testCase.verifyEqual(weightedDistance, plainDistance, 'AbsTol', 1e-12);
            testCase.verifyEqual(weightedH, 2*plainH, 'AbsTol', 1e-12);
        end
    end

    methods (Static, Access = private)
        function planner = smallPlanner()
            % 复用工程全部字段，测试仅缩短时域和控制集，不修改配置文件。
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root, 'config', 'planner_config.json')));
            % 历史Day6用例固定为原前进/三转角控制，新增倒车由独立套件验收。
            planner.vhybrid.reverse_enabled = false;
            planner.vhybrid.control_steering_samples_rad = ...
                [-planner.dynamics.delta_max_rad,0,planner.dynamics.delta_max_rad];
            planner.st_occupancy.x_min = 0;
            planner.st_occupancy.x_max = 30;
            planner.st_occupancy.y_min = 0;
            planner.st_occupancy.y_max = 20;
            planner.st_occupancy.t_min = 0;
            planner.st_occupancy.t_max = 10;
            planner.st_occupancy.dx = 0.5;
            planner.st_occupancy.dy = 0.5;
            planner.st_occupancy.dt = 0.5;
            planner.st_occupancy.safety_distance_m = 0;
            planner.st_occupancy.sweep_max_dt_s = 0.05;
            planner.st_occupancy.sweep_max_distance_m = 0.1;
            planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
            planner.vhybrid.time_step_s = 0.5;
            planner.vhybrid.time_grid_resolution_s = 0.5;
            planner.day6 = struct('collision_sample_step_s', 0.05, ...
                'collision_sample_step_m', 0.1, 'minimum_safety_distance_m', 0, ...
                'goal_hold_enabled', true, 'require_immediate_start', true, ...
                'max_conflict_examples', 8);
        end

        function vehicle = smallVehicle()
            % 测试车后轴坐标：前1m、后0.5m、宽0.4m；角度rad。
            dimensions = struct('width', 0.4, 'length', 1.5, 'wheelbase', 0.6, ...
                'rear_axle_to_rear', 0.5, 'front_axle_to_front', 0.4);
            vehicle = struct('dimensions_m', dimensions, ...
                'collision_margin_m', struct('front', 0, 'rear', 0, 'side', 0), ...
                'steering', struct('maximum_steer_rad', 0.4680017179));
        end

        function vehicle = tinyVehicle()
            % 小车使整段中间碰撞和粗栅格保守冲突可独立观察。
            vehicle = test_dynamic_collision.smallVehicle();
            vehicle.dimensions_m.width = 0.1;
            vehicle.dimensions_m.length = 0.2;
            vehicle.dimensions_m.wheelbase = 0.06;
            vehicle.dimensions_m.front_axle_to_front = 0.04;
            vehicle.dimensions_m.rear_axle_to_rear = 0.1;
        end

        function vehicle = actualVehicle()
            root = fileparts(fileparts(mfilename('fullpath')));
            vehicle = jsondecode(fileread(fullfile(root, 'config', 'vehicle_config.json')));
        end

        function geometry = openGeometry()
            geometry = struct('boundary_xy', [0, 0; 30, 0; 30, 20; 0, 20; 0, 0], ...
                'obstacles', {{}}, 'bounds', [0, 0, 30, 20]);
        end

        function state = integrate(start, steering, acceleration, dt, vehicle, planner)
            % 通过 Day3 公开自行车模型获得真值，避免在测试里复制方程。
            d = planner.dynamics;
            [state, ~, ~, valid] = summon_vehicle_dynamic(start, steering, acceleration, ...
                dt, vehicle.dimensions_m.wheelbase, d.v_max_mps, d.v_min_mps, ...
                d.a_min_mps2, d.a_max_mps2, vehicle.steering.maximum_steer_rad);
            assert(valid, 'DynamicCollisionTest:InvalidFixture', '测试运动学输入必须有效。');
        end

        function states = motionStates(motion)
            % 公开接口允许Nx5与统一path；辅助转换不改变状态或时间顺序。
            if isnumeric(motion)
                states = motion;
            else
                states = [motion.x(:), motion.y(:), motion.theta(:), motion.v(:), motion.t(:)];
            end
        end

        function residuals = connectionReplayResiduals(start, nodes, vehicle, planner)
            % 按生成节点控制重放各边，仅辅助转换用循环，不复制自行车模型方程。
            residuals = zeros(numel(nodes), 5);
            previous = start;
            for k = 1:numel(nodes)
                actual = [nodes(k).x, nodes(k).y, nodes(k).theta, nodes(k).v, nodes(k).t];
                expected = test_dynamic_collision.integrate(previous, ...
                    nodes(k).steering_angle, nodes(k).acceleration, ...
                    actual(5)-previous(5), vehicle, planner);
                delta = actual-expected;
                delta(3) = mod(delta(3)+pi, 2*pi)-pi;
                residuals(k, :) = abs(delta);
                previous = actual;
            end
        end
    end
end
