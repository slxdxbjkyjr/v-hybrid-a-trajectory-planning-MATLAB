classdef test_vhybrid_multi_phase < matlab.unittest.TestCase
    %TEST_VHYBRID_MULTI_PHASE 方案C公开接口、真实Day7资源及缺省兼容验收。
    % 测试只读既有搜索结果，不重写Day6/Day7基线或生成假轨迹。
    properties
        Planner
        Vehicle
        Geometry
        Parent
        Goal
        ResourceMap
        Higher
    end
    properties (TestParameter)
        invalidProfile=struct('numeric',1,'vector',[true false],'empty',[])
        invalidSpeed=struct('text','3','matrix',ones(2),'empty',[], ...
            'negative',-1,'nan',NaN,'complex',1+1i)
        invalidFraction=struct('zero',0,'one',1,'nan',NaN,'matrix',ones(2))
        invalidWait=struct('negative',-1,'text','1','nan',NaN,'matrix',ones(2))
    end
    methods (TestClassSetup)
        function loadActualScenario(testCase)
            project=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(project,'matlab','single_vehicle')));
            payload=load(fullfile(project,'outputs','day7_four_vehicle', ...
                'day7_four_vehicle_trajectories.mat'),'paths','result');
            testCase.assertTrue(payload.result.valid);
            ids={payload.result.per_vehicle.id};ego=find(strcmp(ids,'vehicle_002'),1);
            original=payload.paths{ego};i=find(abs(original.t-4)<1e-8,1);
            testCase.assertNotEmpty(i);
            vehicle=jsondecode(fileread(fullfile(project,'config','vehicle_config_day7.json')));
            planner=jsondecode(fileread(fullfile(project,'config','planner_config_day7.json')));
            cfg=jsondecode(fileread(fullfile(project,'config','day7_plan_c_experiment.json')));
            geometry=summon_map(jsondecode(fileread(fullfile(project,'config','map_config_day7.json'))));
            planner.vhybrid.heuristic_weight=planner.day6.search_heuristic_weight;
            planner.vhybrid.heuristic_method=planner.day6.heuristic_method;
            planner.vhybrid.minimum_turning_radius_m=vehicle.dimensions_m.wheelbase/ ...
                tan(min(vehicle.steering.maximum_steer_rad,planner.dynamics.delta_max_rad));
            planner.vhybrid.goal_connection_selection='lowest_cost';
            planner.vhybrid.goal_connection_cost_pruning_enabled=true;
            planner.vhybrid.goal_connection_multi_phase_profile=true;
            planner.vhybrid.goal_connection_multi_phase_cruise_speeds_mps= ...
                cfg.vhybrid.goal_connection_multi_phase_cruise_speeds_mps;
            planner.vhybrid.goal_connection_multi_phase_stop_distance_fractions= ...
                cfg.vhybrid.goal_connection_multi_phase_stop_distance_fractions;
            planner.vhybrid.goal_connection_multi_phase_wait_times_s= ...
                cfg.vhybrid.goal_connection_multi_phase_wait_times_s;
            testCase.Parent=vhybrid_node([original.x(i),original.y(i), ...
                original.theta(i),original.v(i),original.t(i)], ...
                0,0,0,0,0,zeros(1,5),1,original.gear(i));
            testCase.Goal=[original.x(end),original.y(end),original.theta(end),0,0];
            testCase.Planner=planner;testCase.Vehicle=vehicle;testCase.Geometry=geometry;
            high=find(strcmp(ids,'vehicle_001'),1);
            obstacle=dynamic_vehicle_obstacle(payload.paths{high},vehicle,'vehicle_001',planner);
            map=st_occupancy_map(planner,geometry);map=mark_static_obstacles(map,geometry,vehicle);
            map=mark_trajectory_occupancy(map,payload.paths{high},vehicle,'vehicle_001');
            testCase.ResourceMap=prepare_dynamic_context(map,obstacle);
            testCase.Higher=obstacle;
        end
    end
    methods (Test)
        function disabledMatchesMissingGateExactly(testCase)
            off=testCase.Planner;off.vhybrid.goal_connection_multi_phase_profile=false;
            missing=off;missing.vhybrid=rmfield(missing.vhybrid,'goal_connection_multi_phase_profile');
            [okA,a,sa]=testCase.connect(off);
            [okB,b,sb]=testCase.connect(missing);
            testCase.verifyTrue(okA && okB);
            testCase.verifyEqual(a,b);
            testCase.verifyEqual(sa,sb);
            testCase.verifyEqual(sa.multi_phase_candidates,0);
        end
        function actualConflictAllowsStopThenAccelerate(testCase)
            [ok,nodes,stats]=testCase.connect(testCase.Planner);
            testCase.assertTrue(ok);
            testCase.verifyTrue(stats.multi_phase_selected);
            testCase.verifyGreaterThan(stats.multi_phase_stop_wait_s,0);
            testCase.verifyLessThan(nodes(end).t,11);
            testCase.verifyEqual(nodes(end).v,0,'AbsTol',1e-8);
            testCase.verifyGreaterThan(stats.multi_phase_collision_pruned,0);
            testCase.verifyTrue(any([nodes.v]<=1e-8 & [nodes.t]<nodes(end).t-1));
            testCase.verifyTrue(any(diff([nodes.v])>0));
            path=testCase.nodePath(nodes);
            replay=vhybrid_validate_trajectory(path,testCase.Geometry,testCase.Vehicle,testCase.Planner);
            [hit,~]=check_trajectory_conflict(path,testCase.Higher,testCase.Vehicle, ...
                testCase.Planner,testCase.ResourceMap);
            testCase.verifyTrue(replay.valid);
            testCase.verifyFalse(hit);
        end
        function costPruningMatchesFullEnumeration(testCase)
            off=testCase.Planner;off.vhybrid.goal_connection_cost_pruning_enabled=false;
            [okA,a,sa]=testCase.connect(testCase.Planner);
            [okB,b,sb]=testCase.connect(off);
            testCase.assertTrue(okA && okB);
            testCase.verifyEqual(a,b);
            testCase.verifyTrue(sa.multi_phase_selected && sb.multi_phase_selected);
            testCase.verifyGreaterThan(sa.multi_phase_cost_pruned,0);
            testCase.verifyEqual(sb.multi_phase_cost_pruned,0);
        end
        function missingWaitDefaultsToZero(testCase)
            a=testCase.Planner;a.vhybrid.goal_connection_multi_phase_wait_times_s=0;
            b=a;b.vhybrid=rmfield(b.vhybrid,'goal_connection_multi_phase_wait_times_s');
            [okA,nA,sA]=testCase.connect(a);
            [okB,nB,sB]=testCase.connect(b);
            testCase.verifyEqual(okA,okB);testCase.verifyEqual(nA,nB);testCase.verifyEqual(sA,sB);
        end
        function emptyWaitDefaultsToZero(testCase)
            a=testCase.Planner;a.vhybrid.goal_connection_multi_phase_wait_times_s=0;
            b=a;b.vhybrid.goal_connection_multi_phase_wait_times_s=[];
            [okA,nA,sA]=testCase.connect(a);
            [okB,nB,sB]=testCase.connect(b);
            testCase.verifyEqual(okA,okB);testCase.verifyEqual(nA,nB);testCase.verifyEqual(sA,sB);
        end
        function noConflictKeepsLowerCostSingleProfile(testCase)
            planner=testCase.Planner;
            [ok,nodes,stats]=vhybrid_goal_connection(testCase.Parent,testCase.Goal, ...
                testCase.Geometry,testCase.Vehicle,planner);
            testCase.assertTrue(ok);testCase.verifyFalse(stats.multi_phase_selected);
            testCase.verifyGreaterThan(stats.multi_phase_candidates,0);
            testCase.verifyFalse(any([nodes.v]<=1e-8 & [nodes.t]<nodes(end).t-1e-8));
        end
        function rejectsInvalidGate(testCase,invalidProfile)
            planner=testCase.Planner;planner.vhybrid.goal_connection_multi_phase_profile=invalidProfile;
            testCase.verifyError(@()testCase.connect(planner),'VHybrid:InvalidMultiPhaseProfile');
        end
        function rejectsInvalidCruise(testCase,invalidSpeed)
            planner=testCase.Planner;planner.vhybrid.goal_connection_multi_phase_cruise_speeds_mps=invalidSpeed;
            testCase.verifyError(@()testCase.connect(planner),'VHybrid:InvalidMultiPhaseProfile');
        end
        function rejectsInvalidFraction(testCase,invalidFraction)
            planner=testCase.Planner;planner.vhybrid.goal_connection_multi_phase_stop_distance_fractions=invalidFraction;
            testCase.verifyError(@()testCase.connect(planner),'VHybrid:InvalidMultiPhaseProfile');
        end
        function rejectsInvalidWait(testCase,invalidWait)
            planner=testCase.Planner;planner.vhybrid.goal_connection_multi_phase_wait_times_s=invalidWait;
            testCase.verifyError(@()testCase.connect(planner),'VHybrid:InvalidMultiPhaseProfile');
        end
        function positiveMinimumSpeedCannotStop(testCase)
            planner=testCase.Planner;planner.dynamics.v_min_mps=.1;
            testCase.verifyError(@()testCase.connect(planner),'VHybrid:InvalidMultiPhaseProfile');
        end
        function missingRequiredConfigurationHasExplicitError(testCase)
            planner=testCase.Planner;
            planner.vhybrid=rmfield(planner.vhybrid,'goal_connection_multi_phase_cruise_speeds_mps');
            testCase.verifyError(@()testCase.connect(planner),'VHybrid:InvalidMultiPhaseProfile');
        end
    end
    methods (Access=private)
        function [ok,nodes,stats]=connect(testCase,planner)
            [ok,nodes,stats]=vhybrid_goal_connection(testCase.Parent,testCase.Goal, ...
                testCase.Geometry,testCase.Vehicle,planner,testCase.ResourceMap,testCase.Higher);
        end
        function path=nodePath(testCase,nodes)
            p=testCase.Parent;
            fields={'x','y','theta','v','t','acceleration','steering_angle', ...
                'gear','is_gear_change','travelled_distance','direction'};
            path=struct();
            for k=1:numel(fields)
                name=fields{k};path.(name)=[p.(name);vertcat(nodes.(name))];
            end
        end
    end
end

