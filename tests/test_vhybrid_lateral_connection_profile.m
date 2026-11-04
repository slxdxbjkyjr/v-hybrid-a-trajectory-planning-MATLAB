classdef test_vhybrid_lateral_connection_profile < matlab.unittest.TestCase
    %TEST_VHYBRID_LATERAL_CONNECTION_PROFILE Verify curvature-aware Dubins profiles.

    properties
        Dynamics
        Vehicle
        Planner
        Map
    end

    methods (TestClassSetup)
        function addSourceAndLoadConfiguration(testCase)
            project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(project,'matlab','single_vehicle')));
            testCase.Dynamics = struct('a_max_mps2',4,'a_min_mps2',-6, ...
                'v_min_mps',0,'v_max_mps',10);
            testCase.Vehicle = jsondecode(fileread(fullfile(project, ...
                'config','vehicle_config_day7.json')));
            testCase.Planner = jsondecode(fileread(fullfile(project, ...
                'config','planner_config_day7_lateral.json')));
            testCase.Map = summon_map(jsondecode(fileread(fullfile(project, ...
                'config','map_config_day7.json'))));
        end
    end

    methods (Test)
        function straightBeforeArcBrakesBeforeLocalLimit(testCase)
            [phases,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [6,4],[0,0.1],10,0,10,testCase.Dynamics,4.9,1e-9);

            testCase.verifyTrue(feasible);
            testCase.verifyEqual(diagnostic.local_speed_limits_mps,[10,7], ...
                'AbsTol',1e-8);
            testCase.verifyLessThanOrEqual(diagnostic.boundary_speeds_mps(2), ...
                diagnostic.local_speed_limits_mps(2)+1e-9);
            testCase.verifyEqual(diagnostic.required_braking_distance_m,4.25, ...
                'AbsTol',1e-9);
            testCase.verifyEqual(diagnostic.available_braking_distance_m,6, ...
                'AbsTol',1e-12);
            testCase.verifyProfile(phases,10,0,10);
            testCase.verifyLessThanOrEqual(diagnostic.profile_piece_peak_speeds_mps, ...
                diagnostic.profile_piece_speed_limits_mps+1e-9);
        end

        function straightAfterArcCanExceedCurveLimit(testCase)
            [~,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [4,6],[0.1,0],3,0,10,testCase.Dynamics,4.9,1e-9);

            testCase.verifyTrue(feasible);
            testCase.verifyLessThanOrEqual(diagnostic.profile_piece_peak_speeds_mps(1), ...
                diagnostic.local_speed_limits_mps(1)+1e-9);
            testCase.verifyGreaterThan(diagnostic.profile_piece_peak_speeds_mps(2), ...
                diagnostic.local_speed_limits_mps(1));
            testCase.verifyLessThanOrEqual(diagnostic.profile_piece_peak_speeds_mps(2), ...
                diagnostic.local_speed_limits_mps(2)+1e-9);
        end

        function consecutiveArcsUseTheirOwnCurvatureLimits(testCase)
            [~,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [3,2,3],[0.1,0.16,0.08],2,0,10,testCase.Dynamics,4.9,1e-9);

            testCase.verifyTrue(feasible);
            testCase.verifyEqual(diagnostic.local_speed_limits_mps, ...
                sqrt(4.9./[0.1,0.16,0.08]),'AbsTol',1e-8);
            testCase.verifyLessThanOrEqual(diagnostic.profile_piece_peak_speeds_mps, ...
                diagnostic.profile_piece_speed_limits_mps+1e-9);
            testCase.verifyLessThanOrEqual(diagnostic.boundary_speeds_mps(3), ...
                diagnostic.local_speed_limits_mps(2)+1e-9);
        end

        function firstArcRejectsParentAlreadyOverLimit(testCase)
            [phases,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [4,6],[0.1,0],8,0,10,testCase.Dynamics,4.9,1e-9);

            testCase.verifyFalse(feasible);
            testCase.verifyEmpty(phases);
            testCase.verifyEqual(diagnostic.reason,'entry_speed_exceeds_curve_limit');
            testCase.verifyGreaterThan(diagnostic.required_braking_distance_m,0);
            testCase.verifyEqual(diagnostic.available_braking_distance_m,0, ...
                'AbsTol',1e-12);
        end

        function connectionStartingOnArcRejectsOverspeedParent(testCase)
            planner = testCase.Planner;
            planner.vhybrid.goal_connection_max_turn_rad = Inf;
            map = struct('boundary_xy',[0,0;100,0;100,100;0,100;0,0], ...
                'bounds',[0,0,100,100],'centerline_xy',zeros(0,2), ...
                'obstacles',{cell(0,1)});
            parent = vhybrid_node([50,50,0,8,0],0,0,0,0,0,[100,100,12,32,0],1,1,false);

            [success,nodes,statistics] = vhybrid_goal_connection(parent, ...
                [50,60,pi/2,0,0],map,testCase.Vehicle,planner);

            testCase.verifyFalse(success);
            testCase.verifyEmpty(nodes);
            testCase.verifyGreaterThan(statistics.lateral_acceleration_pruned,0);
            testCase.verifyGreaterThan(statistics.speed_profile_diagnostics ...
                .reason_counts.entry_speed_exceeds_curve_limit,0);
            testCase.verifyEqual(statistics.speed_profile_diagnostics.last_unreachable ...
                .available_braking_distance_m,0,'AbsTol',1e-12);
        end

        function insufficientStraightBrakingDistanceIsDiagnosed(testCase)
            [phases,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [1,4],[0,0.1],10,0,10,testCase.Dynamics,4.9,1e-9);

            testCase.verifyFalse(feasible);
            testCase.verifyEmpty(phases);
            testCase.verifyEqual(diagnostic.reason,'insufficient_braking_distance');
            testCase.verifyEqual(diagnostic.required_braking_distance_m,4.25, ...
                'AbsTol',1e-9);
            testCase.verifyEqual(diagnostic.available_braking_distance_m,1, ...
                'AbsTol',1e-12);
        end

        function profileReachesZeroAtExactTerminalDistance(testCase)
            [phases,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [6,4],[0,0.1],0,0,10,testCase.Dynamics,4.9,1e-9);

            testCase.verifyTrue(feasible);
            testCase.verifyProfile(phases,0,0,10);
            testCase.verifyLessThanOrEqual(diagnostic.profile_piece_peak_speeds_mps, ...
                diagnostic.profile_piece_speed_limits_mps+1e-9);
        end

        function stopWaitProfileUsesSameCurvatureEnvelope(testCase)
            [phases,feasible,diagnostic] = vhybrid_segmented_speed_phases( ...
                [5,5],[0,0.1],0,0,3,testCase.Dynamics,4.9,1e-9,5,0.5);

            testCase.verifyTrue(feasible);
            testCase.verifyProfile(phases,0,0,10);
            testCase.verifyEqual(nnz(phases(:,1)==0.5 & phases(:,2)==0),1);
            testCase.verifyLessThanOrEqual(diagnostic.profile_piece_peak_speeds_mps, ...
                diagnostic.profile_piece_speed_limits_mps+1e-9);
        end

        function connectorReplaysCurvatureAwareStraightProfile(testCase)
            parent = vhybrid_node([10,10,0,0,0],0,0,0,0,0,[20,20,12,0,0],1,1,false);
            [success,nodes,statistics] = vhybrid_goal_connection(parent, ...
                [14,10,0,0,0],testCase.Map,testCase.Vehicle,testCase.Planner);
            path = testCase.connectionPath(parent,nodes);
            replay = vhybrid_validate_trajectory(path,testCase.Map, ...
                testCase.Vehicle,testCase.Planner);

            testCase.verifyTrue(success);
            testCase.verifyTrue(replay.valid);
            testCase.verifyEqual(nodes(end).v,0,'AbsTol',1e-10);
            testCase.verifyEqual(sum([nodes.travelled_distance]),4,'AbsTol',1e-8);
            testCase.verifyEqual(statistics.speed_profile_diagnostics.enabled,true);
            testCase.verifyGreaterThan(statistics.speed_profile_diagnostics.feasible_count,0);
            testCase.verifyLessThanOrEqual(statistics.speed_profile_diagnostics.selected ...
                .profile_piece_peak_speeds_mps, ...
                statistics.speed_profile_diagnostics.selected.profile_piece_speed_limits_mps+1e-9);
        end

        function multiPhaseConnectorUsesCurvatureProfile(testCase)
            planner = testCase.Planner;
            planner.vhybrid.goal_connection_selection = 'lowest_cost';
            planner.vhybrid.goal_connection_cost_pruning_enabled = false;
            planner.vhybrid.goal_connection_multi_phase_profile = true;
            planner.vhybrid.goal_connection_multi_phase_cruise_speeds_mps = 3;
            planner.vhybrid.goal_connection_multi_phase_stop_distance_fractions = 0.5;
            planner.vhybrid.goal_connection_multi_phase_wait_times_s = 0.5;
            planner.vhybrid.goal_connection_max_turn_rad = Inf;
            map = struct('boundary_xy',[0,0;100,0;100,100;0,100;0,0], ...
                'bounds',[0,0,100,100],'centerline_xy',zeros(0,2), ...
                'obstacles',{cell(0,1)});
            parent = vhybrid_node([50,50,0,0,0],0,0,0,0,0,[100,100,12,0,0],1,1,false);

            [success,nodes,statistics] = vhybrid_goal_connection(parent, ...
                [54,54,pi/2,0,0],map,testCase.Vehicle,planner);
            path = testCase.connectionPath(parent,nodes);
            replay = vhybrid_validate_trajectory(path,map,testCase.Vehicle,planner);

            testCase.verifyTrue(success);
            testCase.verifyNotEmpty(nodes);
            testCase.verifyTrue(statistics.multi_phase_selected);
            testCase.verifyEqual(statistics.multi_phase_stop_wait_s,0.5,'AbsTol',1e-12);
            testCase.verifyTrue(replay.valid);
            testCase.verifyEqual(nodes(end).v,0,'AbsTol',1e-10);
            testCase.verifyGreaterThan(statistics.multi_phase_candidates,0);
            testCase.verifyGreaterThanOrEqual(statistics.speed_profile_diagnostics.attempt_count, ...
                statistics.candidates);
            testCase.verifyEqual(statistics.speed_profile_diagnostics.last_attempt.candidate_kind, ...
                'multi_phase');
            testCase.verifyTrue(any(statistics.speed_profile_diagnostics.last_attempt ...
                .segment_curvatures_per_m>0));
            testCase.verifyLessThan(min(statistics.speed_profile_diagnostics.last_attempt ...
                .local_speed_limits_mps),planner.dynamics.v_max_mps);
            testCase.verifyLessThanOrEqual(statistics.speed_profile_diagnostics.last_attempt ...
                .profile_piece_peak_speeds_mps, ...
                statistics.speed_profile_diagnostics.last_attempt ...
                .profile_piece_speed_limits_mps+1e-9);
            testCase.verifyFalse(isfield(statistics.speed_profile_diagnostics.reason_counts, ...
                'entry_speed_exceeds_curve_limit'));
        end
    end

    methods (Access=private)
        function verifyProfile(testCase,phases,v0,vf,distance)
            speeds = [v0;v0+cumsum(phases(:,1).*phases(:,2))];
            profile_distance = sum(0.5*(speeds(1:end-1)+speeds(2:end)).*phases(:,1));
            testCase.verifyEqual(speeds(end),vf,'AbsTol',1e-8);
            testCase.verifyEqual(profile_distance,distance,'AbsTol',1e-8);
            testCase.verifyGreaterThanOrEqual(min(speeds), ...
                testCase.Dynamics.v_min_mps-1e-9);
            testCase.verifyLessThanOrEqual(max(speeds), ...
                testCase.Dynamics.v_max_mps+1e-9);
        end

        function path = connectionPath(~,parent,nodes)
            path = struct('x',[parent.x;vertcat(nodes.x)], ...
                'y',[parent.y;vertcat(nodes.y)], ...
                'theta',[parent.theta;vertcat(nodes.theta)], ...
                'v',[parent.v;vertcat(nodes.v)],'t',[parent.t;vertcat(nodes.t)], ...
                'acceleration',[0;vertcat(nodes.acceleration)], ...
                'steering_angle',[0;vertcat(nodes.steering_angle)], ...
                'gear',[parent.gear;vertcat(nodes.gear)], ...
                'is_gear_change',[false;vertcat(nodes.is_gear_change)]);
        end
    end
end
