classdef test_lateral_acceleration < matlab.unittest.TestCase
    %TEST_LATERAL_ACCELERATION Public lateral-limit and legacy regression tests.
    % Constant controls retain the existing average-speed arc integration.
    % Deliberately infeasible paths are generated with the disabled legacy model;
    % acceptance then recomputes every segment from its actual entering control.

    properties (TestParameter)
        passingCase = struct( ...
            'straight',struct('v0',10,'v1',10,'delta',0,'expected',0), ...
            'left',struct('v0',2,'v1',3,'delta',atan(0.278),'expected',0.9), ...
            'right',struct('v0',2,'v1',3,'delta',-atan(0.278),'expected',0.9), ...
            'reverseLeft',struct('v0',-2,'v1',-3,'delta',atan(0.278),'expected',0.9), ...
            'reverseRight',struct('v0',-2,'v1',-3,'delta',-atan(0.278),'expected',0.9), ...
            'critical',struct('v0',7,'v1',7,'delta',atan(0.278),'expected',4.9));
        invalidLimit = struct('zero',0,'negative',-4.9,'nan',NaN, ...
            'infinity',Inf,'complex',4.9+1i,'vector',[4.9,4.9],'text','4.9','logical',true);
        invalidTolerance = struct('negative',-1e-9,'nan',NaN, ...
            'infinity',Inf,'complex',1e-9+1i,'vector',[1e-9,1e-9], ...
            'text','1e-9','tooLarge',1e-3);
        invalidEnable = struct('text','true','numeric',1,'vector',[true,false]);
        invalidHeading = struct('nan',NaN,'positiveInfinity',Inf,'negativeInfinity',-Inf);
        gearCase = struct('forward',1,'reverse',-1);
        day7Vehicle = struct('vehicle001',1,'vehicle002',2,'vehicle003',3,'vehicle004',4);
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(project,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function restrictedDay7PlannerOnlyContainsAuthorizedDifferences(testCase)
            original = testCase.readConfiguration('planner_config_day7.json');
            restricted = testCase.readConfiguration('planner_config_day7_lateral.json');
            expected = original;
            expected.stage = 'day7_lateral_constraint_task1';
            expected.output_directory = 'outputs/day7_lateral_constraint';
            expected.dynamics.lateral_acceleration_constraint_enabled = true;
            expected.dynamics.max_lateral_acceleration_mps2 = 4.9;
            expected.dynamics.lateral_acceleration_tolerance_mps2 = 1e-9;
            expected.day6.require_immediate_start = false;
            config = validate_lateral_acceleration_config(restricted.dynamics);
            testCase.verifyEqual(testCase.canonicalStructure(restricted), ...
                testCase.canonicalStructure(expected));
            testCase.verifyTrue(config.enabled);
            testCase.verifyEqual(config.max_lateral_acceleration_mps2,4.9,'AbsTol',1e-12);
            testCase.verifyEqual(config.lateral_acceleration_tolerance_mps2,1e-9,'AbsTol',1e-15);
            testCase.verifyFalse(restricted.day6.require_immediate_start);
            testCase.verifyFalse(restricted.trajectory_optimization_enabled);
            testCase.verifyTrue(original.day6.require_immediate_start);
            testCase.verifyFalse(isfield(original.dynamics, ...
                'lateral_acceleration_constraint_enabled'));
        end

        function restrictedDay7ScenarioOnlyRedirectsMetadata(testCase)
            original = testCase.readConfiguration('scenario_day7_four_vehicle.json');
            restricted = testCase.readConfiguration('scenario_day7_lateral.json');
            expected = original;
            expected.scenario_id = 'day7_lateral_constraint';
            expected.description = ['横向硬约束独立配置：四车t=0且v=0，允许搜索选择等待后起步，' ...
                '固定优先级001→002→004→003。'];
            expected.planner_config = 'planner_config_day7_lateral.json';
            testCase.verifyEqual(testCase.canonicalStructure(restricted), ...
                testCase.canonicalStructure(expected));
            testCase.verifyEqual(restricted.vehicles,original.vehicles);
            testCase.verifyEqual(restricted.map_config,'map_config_day7.json');
            testCase.verifyEqual(restricted.vehicle_config,'vehicle_config_day7.json');
            testCase.verifyEqual(restricted.static_obstacles,original.static_obstacles);
        end

        function restrictedDay7ScenarioKeepsActualPriorityAndPoses(testCase)
            scenario = testCase.readConfiguration('scenario_day7_lateral.json');
            [starts,goals] = testCase.configuredPoses(scenario);
            [~,order] = sort([scenario.vehicles.priority]);
            testCase.verifyEqual({scenario.vehicles.id}, ...
                {'vehicle_001','vehicle_002','vehicle_003','vehicle_004'});
            testCase.verifyEqual([scenario.vehicles.priority],[1,2,4,3]);
            testCase.verifyEqual({scenario.vehicles(order).id}, ...
                {'vehicle_001','vehicle_002','vehicle_004','vehicle_003'});
            testCase.verifyEqual(starts,[4,25,-1.57;8,26,0;27,25,3.14;23,20,-1.57], ...
                'AbsTol',1e-12);
            testCase.verifyEqual(goals,[25,5,-1.57;20,6,-1.57;6,4,3.14;6,10,-3.14], ...
                'AbsTol',1e-12);
        end

        function restrictedDay7RetainsVehicleAndSteeringLimits(testCase)
            scenario = testCase.readConfiguration('scenario_day7_lateral.json');
            planner = testCase.readConfiguration(scenario.planner_config);
            vehicle = testCase.readConfiguration(scenario.vehicle_config);
            testCase.verifyEqual(vehicle.vehicle_model_id,'F515_APA_Seres_V3p24');
            testCase.verifyEqual([vehicle.dimensions_m.width,vehicle.dimensions_m.length, ...
                vehicle.dimensions_m.wheelbase,vehicle.dimensions_m.rear_axle_to_rear, ...
                vehicle.dimensions_m.front_axle_to_front, ...
                vehicle.dimensions_m.front_overhang_plus_wheelbase], ...
                [1.855,4.763,2.78,1.007,0.976,3.756],'AbsTol',1e-12);
            testCase.verifyEqual(vehicle.steering.maximum_steer_rad,0.4680017179,'AbsTol',1e-12);
            testCase.verifyEqual([vehicle.collision_margin_m.front, ...
                vehicle.collision_margin_m.rear,vehicle.collision_margin_m.side], ...
                [0.2,0.2,0.2],'AbsTol',1e-12);
            testCase.verifyEqual([planner.dynamics.v_min_mps,planner.dynamics.v_max_mps, ...
                planner.dynamics.a_min_mps2,planner.dynamics.a_max_mps2, ...
                planner.dynamics.delta_max_rad,planner.dynamics.direction_change_time_s], ...
                [0,10,-6,4,0.4680017179,0.5],'AbsTol',1e-12);
        end

        function restrictedDay7StartsAtZeroAndAllowsAWait(testCase,day7Vehicle)
            % Configuration/acceptance test only: no search or new search result.
            scenario = testCase.readConfiguration('scenario_day7_lateral.json');
            planner = testCase.readConfiguration(scenario.planner_config);
            vehicle = testCase.readConfiguration(scenario.vehicle_config);
            geometry = summon_map(testCase.readConfiguration(scenario.map_config));
            pose = scenario.vehicles(day7Vehicle).start_pose;
            start = [pose.x_m,pose.y_m,pose.yaw_rad,planner.vhybrid.initial_speed_mps,0];
            path = testCase.integratePath(start,[0,0,0.5,1;0,3,0.5,1],vehicle,planner);
            report = vhybrid_validate_trajectory(path,geometry,vehicle,planner);
            testCase.verifyEqual(planner.vhybrid.initial_speed_mps,0,'AbsTol',1e-12);
            testCase.verifyEqual(path.t,[0;0.5;1],'AbsTol',1e-12);
            testCase.verifyEqual(path.v,[0;0;1.5],'AbsTol',1e-12);
            testCase.verifyEqual([path.x(2),path.y(2),path.theta(2)],start(1:3),'AbsTol',1e-12);
            testCase.verifyFalse(planner.day6.require_immediate_start);
            testCase.verifyTrue(report.valid);
            testCase.verifyTrue(report.lateral_acceleration.enabled);
            testCase.verifyEqual(report.lateral_acceleration.violation_count,0);
        end

        function unifiedKernelAcceptsExpectedCases(testCase,passingCase)
            [valid,detail] = summon_lateral_acceleration(passingCase.v0, ...
                passingCase.v1,passingCase.delta,2.78,testCase.dynamics());
            testCase.verifyTrue(valid);
            testCase.verifyEqual(detail.peak_abs_ay_mps2,passingCase.expected,'AbsTol',1e-12);
            testCase.verifyEqual(detail.curvature_per_m, ...
                tan(passingCase.delta)/2.78,'AbsTol',1e-12);
        end

        function enabledConfigurationIsNormalized(testCase)
            config = validate_lateral_acceleration_config(testCase.dynamics());
            testCase.verifyTrue(config.enabled);
        end

        function enabledMissingLimitErrors(testCase)
            dynamics = rmfield(testCase.dynamics(),'max_lateral_acceleration_mps2');
            testCase.verifyError(@() validate_lateral_acceleration_config(dynamics), ...
                'LateralAcceleration:InvalidConfig');
        end

        function enabledIllegalLimitErrors(testCase,invalidLimit)
            dynamics = testCase.dynamics();
            dynamics.max_lateral_acceleration_mps2 = invalidLimit;
            testCase.verifyError(@() validate_lateral_acceleration_config(dynamics), ...
                'LateralAcceleration:InvalidConfig');
        end

        function enabledIllegalToleranceErrors(testCase,invalidTolerance)
            dynamics = testCase.dynamics();
            dynamics.lateral_acceleration_tolerance_mps2 = invalidTolerance;
            testCase.verifyError(@() validate_lateral_acceleration_config(dynamics), ...
                'LateralAcceleration:InvalidConfig');
        end

        function illegalEnableFieldErrors(testCase,invalidEnable)
            dynamics = testCase.dynamics();
            dynamics.lateral_acceleration_constraint_enabled = invalidEnable;
            testCase.verifyError(@() validate_lateral_acceleration_config(dynamics), ...
                'LateralAcceleration:InvalidConfig');
        end

        function disabledMissingLimitIsCompatible(testCase)
            dynamics = struct('lateral_acceleration_constraint_enabled',false);
            config = validate_lateral_acceleration_config(dynamics);
            testCase.verifyFalse(config.enabled);
            [valid,~] = summon_lateral_acceleration(10,10,0.468,2.78,dynamics);
            testCase.verifyTrue(valid);
        end

        function disabledIllegalLimitIsIgnored(testCase)
            dynamics = testCase.dynamics();
            dynamics.lateral_acceleration_constraint_enabled = false;
            dynamics.max_lateral_acceleration_mps2 = NaN;
            dynamics.lateral_acceleration_tolerance_mps2 = -1;
            config = validate_lateral_acceleration_config(dynamics);
            [valid,~] = summon_lateral_acceleration(10,10,0.468,2.78,dynamics);
            testCase.verifyFalse(config.enabled);
            testCase.verifyTrue(valid);
        end

        function missingToleranceUsesRoundoffDefault(testCase)
            dynamics = rmfield(testCase.dynamics(),'lateral_acceleration_tolerance_mps2');
            config = validate_lateral_acceleration_config(dynamics);
            testCase.verifyTrue(config.enabled);
            testCase.verifyEqual(config.lateral_acceleration_tolerance_mps2,1e-9,'AbsTol',1e-15);
        end

        function zeroToleranceIsAccepted(testCase)
            dynamics = testCase.dynamics();
            dynamics.lateral_acceleration_tolerance_mps2 = 0;
            config = validate_lateral_acceleration_config(dynamics);
            testCase.verifyTrue(config.enabled);
            testCase.verifyEqual(config.lateral_acceleration_tolerance_mps2,0,'AbsTol',1e-15);
        end

        function missingEnableFieldPreservesLegacyBehavior(testCase)
            dynamics = rmfield(testCase.dynamics(),'lateral_acceleration_constraint_enabled');
            config = validate_lateral_acceleration_config(dynamics);
            [valid,detail] = summon_lateral_acceleration(10,10,0.468,2.78,dynamics);
            testCase.verifyFalse(config.enabled);
            testCase.verifyTrue(valid);
            testCase.verifyGreaterThan(detail.peak_abs_ay_mps2,4.9);
        end

        function roundoffToleranceAcceptsOnlyNearBoundary(testCase)
            [valid,detail] = summon_lateral_acceleration( ...
                sqrt((4.9+0.5e-9)/0.1),7,atan(0.278),2.78,testCase.dynamics());
            testCase.verifyTrue(valid);
            testCase.verifyGreaterThan(detail.peak_abs_ay_mps2,4.9);
            testCase.verifyLessThanOrEqual(detail.peak_abs_ay_mps2,4.9+1e-9);
        end

        function justBeyondToleranceIsRejected(testCase)
            [valid,detail] = summon_lateral_acceleration( ...
                sqrt((4.9+4e-9)/0.1),7,atan(0.278),2.78,testCase.dynamics());
            testCase.verifyFalse(valid);
            testCase.verifyGreaterThan(detail.peak_abs_ay_mps2,4.9+1e-9);
        end

        function highSpeedTurnIsRejected(testCase)
            [valid,detail] = summon_lateral_acceleration(10,10,0.468,2.78,testCase.dynamics());
            testCase.verifyFalse(valid);
            testCase.verifyGreaterThan(detail.peak_abs_ay_mps2,4.9);
            testCase.verifyNotEmpty(detail.reason);
        end

        function decelerationChecksEntranceNotOnlyExit(testCase)
            [valid,detail] = summon_lateral_acceleration(8,6,atan(0.278),2.78,testCase.dynamics());
            testCase.verifyFalse(valid);
            testCase.verifyEqual(detail.entry_abs_ay_mps2,6.4,'AbsTol',1e-12);
            testCase.verifyEqual(detail.exit_abs_ay_mps2,3.6,'AbsTol',1e-12);
            testCase.verifyEqual(detail.peak_abs_ay_mps2,6.4,'AbsTol',1e-12);
        end

        function signedReverseDecelerationChecksEntrance(testCase)
            [valid,detail] = summon_lateral_acceleration(-8,-6,-atan(0.278),2.78,testCase.dynamics());
            testCase.verifyFalse(valid);
            testCase.verifyEqual(detail.peak_abs_ay_mps2,6.4,'AbsTol',1e-12);
            testCase.verifyEqual(detail.curvature_per_m,-0.1,'AbsTol',1e-12);
        end

        function oldTenArgumentInterfaceIsUnchanged(testCase)
            start = [0,0,0,8,0];
            [actual,ds,kappa,valid] = summon_vehicle_dynamic( ...
                start,atan(0.278),-2,1,2.78,10,0,-6,4,0.4680017179);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(ds,7,'AbsTol',1e-12);
            testCase.verifyEqual(kappa,0.1,'AbsTol',1e-12);
            testCase.verifyEqual(actual(4:5),[6,1],'AbsTol',1e-12);
            testCase.verifyEqual(actual(1:3),[10*sin(0.7),10*(1-cos(0.7)),0.7],'AbsTol',1e-12);
        end

        function enabledOptionalArgumentRejectsEntireAction(testCase)
            start = [0,0,0,8,0];
            [actual,ds,kappa,valid] = summon_vehicle_dynamic( ...
                start,atan(0.278),-2,1,2.78,10,0,-6,4,0.4680017179,testCase.dynamics());
            testCase.verifyFalse(valid);
            testCase.verifyEqual(actual,start,'AbsTol',1e-12);
            testCase.verifyEqual([ds,kappa],[0,0],'AbsTol',1e-12);
        end

        function optionalMissingEnableMatchesTenArguments(testCase)
            dynamics = rmfield(testCase.dynamics(),'lateral_acceleration_constraint_enabled');
            start = [0,0,0,8,0];
            [actual,ds,kappa,valid] = summon_vehicle_dynamic( ...
                start,atan(0.278),-2,1,2.78,10,0,-6,4,0.4680017179,dynamics);
            [expected,expectedDs,expectedKappa,expectedValid] = summon_vehicle_dynamic( ...
                start,atan(0.278),-2,1,2.78,10,0,-6,4,0.4680017179);
            testCase.verifyEqual(valid,expectedValid);
            testCase.verifyEqual(actual,expected,'AbsTol',1e-12);
            testCase.verifyEqual([ds,kappa],[expectedDs,expectedKappa],'AbsTol',1e-12);
        end

        function optionalArgumentFailsForInvalidEnabledConfig(testCase)
            dynamics = rmfield(testCase.dynamics(),'max_lateral_acceleration_mps2');
            testCase.verifyError(@() summon_vehicle_dynamic([0,0,0,0,0], ...
                0,1,1,2.78,10,0,-6,4,0.4680017179,dynamics), ...
                'LateralAcceleration:InvalidConfig');
        end

        function gearAdapterRejectsUnsafeEntrance(testCase,gearCase)
            start = [0,0,0,8,0];
            [actual,ds,kappa,valid] = summon_vehicle_dynamic_gear(start,atan(0.278), ...
                -2,1,testCase.vehicle(),testCase.planner(),gearCase);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(actual,start,'AbsTol',1e-12);
            testCase.verifyEqual([ds,kappa],[0,0],'AbsTol',1e-12);
        end

        function gearAdapterAcceptsSafeForwardAndReverse(testCase,gearCase)
            [actual,ds,kappa,valid] = summon_vehicle_dynamic_gear([0,0,0,2,0], ...
                atan(0.278),1,1,testCase.vehicle(),testCase.planner(),gearCase);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(actual(4:5),[3,1],'AbsTol',1e-12);
            testCase.verifyEqual(ds,gearCase*2.5,'AbsTol',1e-12);
            testCase.verifyEqual(kappa,0.1,'AbsTol',1e-12);
            testCase.verifyEqual(actual(3),gearCase*0.25,'AbsTol',1e-12);
        end

        function gearAdapterKeepsLegacyConfiguration(testCase,gearCase)
            planner = testCase.legacyPlanner();
            [actual,ds,~,valid] = summon_vehicle_dynamic_gear([0,0,0,8,0], ...
                atan(0.278),-2,1,testCase.vehicle(),planner,gearCase);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(actual(4:5),[6,1],'AbsTol',1e-12);
            testCase.verifyEqual(ds,gearCase*7,'AbsTol',1e-12);
        end

        function gearAdapterErrorsOnMissingEnabledLimit(testCase)
            planner = testCase.planner();
            planner.dynamics = rmfield(planner.dynamics,'max_lateral_acceleration_mps2');
            testCase.verifyError(@() summon_vehicle_dynamic_gear([0,0,0,0,0], ...
                0,1,1,testCase.vehicle(),planner,1),'LateralAcceleration:InvalidConfig');
        end

        function enabledModelRetainsAverageSpeedIntegration(testCase)
            [actual,ds,kappa,valid] = summon_vehicle_dynamic([0,0,0,2,0], ...
                atan(0.278),1,1,2.78,10,0,-6,4,0.4680017179,testCase.dynamics());
            testCase.verifyTrue(valid);
            testCase.verifyEqual(ds,2.5,'AbsTol',1e-12);
            testCase.verifyEqual(kappa,0.1,'AbsTol',1e-12);
            testCase.verifyEqual(actual,[10*sin(0.25),10*(1-cos(0.25)),0.25,3,1], ...
                'AbsTol',1e-12);
        end

        function validatorReportsNewSteeringEntranceViolation(testCase)
            path = testCase.path([0,0,0,8,0],[0,0,1,1;atan(0.278),-2,1,1]);
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            lateral = report.lateral_acceleration;
            testCase.verifyFalse(report.valid);
            testCase.verifyGreaterThanOrEqual(report.constraint_failures,1);
            testCase.verifyEqual(lateral.violation_count,1);
            testCase.verifyEqual(lateral.violations.segment_index,2);
            testCase.verifyEqual(lateral.violations.reason,'entry_lateral_acceleration_exceeded');
            testCase.verifyEqual(lateral.violations.peak_abs_ay_mps2,6.4,'AbsTol',1e-12);
            testCase.verifyEqual(lateral.violations.peak_time_s,1,'AbsTol',1e-12);
            testCase.verifyEqual(lateral.violations.peak_position_xy,[8,0],'AbsTol',1e-12);
            testCase.verifyEqual(lateral.violations.peak_speed_mps,8,'AbsTol',1e-12);
            testCase.verifyEqual(lateral.violations.curvature_per_m,0.1,'AbsTol',1e-12);
        end

        function validatorUsesControlsInsteadOfStoredCurvature(testCase)
            path = testCase.path([0,0,0,8,0],[atan(0.278),-2,1,1]);
            path.curvature = zeros(size(path.t));
            report = vhybrid_validate_lateral_acceleration(path,testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.violation_count,1);
            testCase.verifyEqual(report.violations.curvature_per_m,0.1,'AbsTol',1e-12);
        end

        function validatorReportsAcceleratingExitPeak(testCase)
            path = testCase.path([0,0,0,5,0],[atan(0.278),3,1,1]);
            report = vhybrid_validate_lateral_acceleration(path,testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.violations.reason,'exit_lateral_acceleration_exceeded');
            testCase.verifyEqual(report.violations.peak_abs_ay_mps2,6.4,'AbsTol',1e-12);
            testCase.verifyEqual(report.violations.peak_time_s,1,'AbsTol',1e-12);
            testCase.verifyEqual(report.violations.peak_position_xy,[path.x(end),path.y(end)],'AbsTol',1e-12);
            testCase.verifyEqual(report.violations.peak_speed_mps,8,'AbsTol',1e-12);
        end

        function validatorDoesNotTrustClippedStoredEndSpeed(testCase)
            path = testCase.path([0,0,0,5,0],[atan(0.278),3,1,1]);
            path.v(end) = 6;
            report = vhybrid_validate_lateral_acceleration(path,testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.violations.reason,'model_exit_lateral_acceleration_exceeded');
            testCase.verifyEqual(report.violations.peak_abs_ay_mps2,6.4,'AbsTol',1e-12);
        end

        function validatorChecksReverseEntrance(testCase)
            path = testCase.path([0,0,0,8,0],[-atan(0.278),-2,1,-1]);
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.lateral_acceleration.violation_count,1);
            testCase.verifyEqual(report.lateral_acceleration.violations.peak_abs_ay_mps2,6.4,'AbsTol',1e-12);
        end

        function validatorAcceptsSafeControlledTrajectory(testCase)
            path = testCase.path([0,0,0,2,0],[atan(0.278),1,1,1;-atan(0.278),-1,1,1]);
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyTrue(report.valid);
            testCase.verifyTrue(report.lateral_acceleration.valid);
            testCase.verifyEqual(report.lateral_acceleration.checked_segment_count,2);
            testCase.verifyEqual(report.lateral_acceleration.violation_count,0);
            testCase.verifyEqual(report.lateral_acceleration.max_peak_abs_ay_mps2,0.9,'AbsTol',1e-12);
        end

        function validatorReportsMissingHeadingWithoutThrowing(testCase)
            path = rmfield(testCase.path([0,0,0,2,0],[0,0,0.5,1]),'theta');
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.constraint_failures,1);
            testCase.verifyEqual(report.lateral_acceleration.reason,'missing_or_invalid_path_fields');
            testCase.verifyEqual(report.failure_reasons,{'missing_or_invalid_path_fields'});
        end

        function validatorReportsShortHeadingWithoutThrowing(testCase)
            path = testCase.path([0,0,0,2,0],[0,0,0.5,1]);
            path.theta = 0;
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.constraint_failures,1);
            testCase.verifyEqual(report.lateral_acceleration.reason,'invalid_trajectory_field_shape_or_type');
            testCase.verifyEqual(report.failure_reasons,{'invalid_trajectory_field_shape_or_type'});
        end

        function validatorReportsNonfiniteHeadingWithoutThrowing(testCase,invalidHeading)
            path = testCase.path([0,0,0,2,0],[0,0,0.5,1]);
            path.theta(2) = invalidHeading;
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.constraint_failures,1);
            testCase.verifyEqual(report.lateral_acceleration.invalid_segment_count,1);
            testCase.verifyEqual(report.failure_reasons,{'invalid_segment_data_or_duration'});
        end

        function validatorReportsCellGearChangeFlagsWithoutThrowing(testCase)
            path = testCase.path([0,0,0,2,0],[0,0,0.5,1]);
            path.is_gear_change = {false;false};
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.constraint_failures,1);
            testCase.verifyEqual(report.gear_change_failures,1);
            testCase.verifyEqual(report.failure_reasons,{'invalid_gear_change_flags'});
        end

        function validatorNormalizesMixedIntegerTrajectoryFields(testCase)
            path = testCase.path([0,0,0,2,0],[0,0,0.5,1]);
            path.y = int16(path.y);
            path.theta = int16(path.theta);
            path.v = uint8(path.v);
            path.acceleration = int8(path.acceleration);
            path.gear = int8(path.gear);
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.planner());
            testCase.verifyTrue(report.valid);
            testCase.verifyEqual(report.max_position_residual_m,0,'AbsTol',1e-12);
            testCase.verifyEqual(report.max_speed_residual_mps,0,'AbsTol',1e-12);
            testCase.verifyEqual(report.lateral_acceleration.checked_segment_count,1);
        end

        function validatorLegacyModeDoesNotEnforceNewLimit(testCase)
            path = testCase.path([0,0,0,8,0],[atan(0.278),-2,1,1]);
            report = vhybrid_validate_trajectory(path,testCase.map(),testCase.vehicle(),testCase.legacyPlanner());
            testCase.verifyTrue(report.valid);
            testCase.verifyFalse(report.lateral_acceleration.enabled);
        end
    end

    methods (Static,Access=private)
        function config = readConfiguration(filename)
            project = fileparts(fileparts(mfilename('fullpath')));
            config = jsondecode(fileread(fullfile(project,'config',filename)));
        end

        function value = canonicalStructure(value)
            % Field order must not affect exact recursive configuration checks.
            if isstruct(value)
                value = orderfields(value);
                names = fieldnames(value);
                for index = 1:numel(value)
                    for field = 1:numel(names)
                        value(index).(names{field}) = test_lateral_acceleration.canonicalStructure( ...
                            value(index).(names{field}));
                    end
                end
            elseif iscell(value)
                for index = 1:numel(value)
                    value{index} = test_lateral_acceleration.canonicalStructure(value{index});
                end
            end
        end

        function [starts,goals] = configuredPoses(scenario)
            count = numel(scenario.vehicles);
            starts = zeros(count,3);
            goals = zeros(count,3);
            for index = 1:count
                vehicle = scenario.vehicles(index);
                starts(index,:) = [vehicle.start_pose.x_m,vehicle.start_pose.y_m, ...
                    vehicle.start_pose.yaw_rad];
                goals(index,:) = [vehicle.summon_goal.x_m,vehicle.summon_goal.y_m, ...
                    vehicle.summon_goal.yaw_rad];
            end
        end

        function dynamics = dynamics()
            dynamics = struct('default_dt_s',0.1,'v_min_mps',0,'v_max_mps',10, ...
                'a_min_mps2',-6,'a_max_mps2',4,'delta_max_rad',0.4680017179, ...
                'direction_change_time_s',0.5, ...
                'lateral_acceleration_constraint_enabled',true, ...
                'max_lateral_acceleration_mps2',4.9, ...
                'lateral_acceleration_tolerance_mps2',1e-9);
        end

        function planner = planner()
            planner = struct('dynamics',test_lateral_acceleration.dynamics(), ...
                'vhybrid',struct('kinematic_validation_tolerance',1e-8));
        end

        function planner = legacyPlanner()
            planner = test_lateral_acceleration.planner();
            planner.dynamics = rmfield(planner.dynamics, ...
                {'lateral_acceleration_constraint_enabled', ...
                'max_lateral_acceleration_mps2','lateral_acceleration_tolerance_mps2'});
        end

        function vehicle = vehicle()
            vehicle = struct('dimensions_m',struct('wheelbase',2.78,'width',1.855, ...
                'length',4.763,'rear_axle_to_rear',1.007,'front_axle_to_front',0.976), ...
                'collision_margin_m',struct('front',0.2,'rear',0.2,'side',0.2), ...
                'steering',struct('maximum_steer_rad',0.4680017179));
        end

        function map = map()
            map = struct('boundary_xy',[-100,-100;100,-100;100,100;-100,100;-100,-100], ...
                'obstacles',{{}},'bounds',[-100,-100,100,100]);
        end

        function path = path(start,controls)
            path = test_lateral_acceleration.integratePath(start,controls, ...
                test_lateral_acceleration.vehicle(),test_lateral_acceleration.legacyPlanner());
        end

        function path = integratePath(start,controls,vehicle,planner)
            count = size(controls,1);
            states = zeros(count+1,5);
            states(1,:) = start;
            for k = 1:count
                [states(k+1,:),~,~,valid] = summon_vehicle_dynamic_gear(states(k,:), ...
                    controls(k,1),controls(k,2),controls(k,3), ...
                    vehicle,planner,controls(k,4));
                assert(valid,'LateralTest:FixtureFailure','Fixture integration failed.');
            end
            path = struct('x',states(:,1),'y',states(:,2),'theta',states(:,3), ...
                'v',states(:,4),'t',states(:,5),'steering_angle',[0;controls(:,1)], ...
                'acceleration',[0;controls(:,2)],'gear',[controls(1,4);controls(:,4)], ...
                'is_gear_change',false(count+1,1),'valid',true);
        end
    end
end
