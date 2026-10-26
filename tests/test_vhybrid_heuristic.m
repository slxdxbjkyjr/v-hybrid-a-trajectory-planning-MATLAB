classdef test_vhybrid_heuristic < matlab.unittest.TestCase
    %TEST_VHYBRID_HEURISTIC 几何下界、包角连续性与既有启发兼容测试。
    % 使用公开接口和已知几何案例；不搜索路径，不依赖场景JSON参数。

    properties (TestParameter)
        geometry = struct( ...
            'samePose',struct('state',[0,0,0,0,0], ...
                'goal',[0,0,0,0,0],'expected',0), ...
            'positionOnly',struct('state',[0,0,0,0,0], ...
                'goal',[3,4,0,0,0],'expected',5), ...
            'headingOnly',struct('state',[0,0,0,0,0], ...
                'goal',[0,0,pi/2,0,0],'expected',5.5*pi/2), ...
            'positionDominates',struct('state',[0,0,0,0,0], ...
                'goal',[6,8,0.1,0,0],'expected',10), ...
            'headingDominates',struct('state',[0,0,0,0,0], ...
                'goal',[1,0,pi/2,0,0],'expected',5.5*pi/2), ...
            'wrapAcrossPi',struct('state',[0,0,pi-0.1,0,0], ...
                'goal',[0,0,-pi+0.1,0,0],'expected',1.1), ...
            'equivalentPi',struct('state',[0,0,pi,0,0], ...
                'goal',[0,0,-pi,0,0],'expected',0), ...
            'fullTurn',struct('state',[0,0,0,0,0], ...
                'goal',[0,0,2*pi,0,0],'expected',0), ...
            'halfTurn',struct('state',[0,0,0,0,0], ...
                'goal',[0,0,-pi,0,0],'expected',5.5*pi))
        gear = struct('forward',1,'reverse',-1)
        reverseEnabled = struct('disabled',false,'enabled',true)
        invalidRadius = struct('zero',0,'negative',-1,'nan',NaN, ...
            'infinite',Inf,'negativeInfinite',-Inf,'empty',[], ...
            'vector',[1,2],'matrix',ones(2),'text','5.5', ...
            'logical',true,'complex',5.5+1i)
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectDir,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function geometricCasesUseMaximumAndSharedWeight(testCase,geometry)
            cfg = testCase.lowerBoundConfig();
            [h,distance,method] = vhybrid_heuristic( ...
                geometry.state,geometry.goal,cfg);

            testCase.verifyEqual(distance,geometry.expected,'AbsTol',1e-11);
            testCase.verifyEqual(h,3*geometry.expected,'AbsTol',1e-11);
            testCase.verifyEqual(method,'kinematic_lower_bound');
        end

        function forwardReverseAndEnableFlagUseSameGeometricBound( ...
                testCase,gear,reverseEnabled)
            cfg = testCase.lowerBoundConfig();
            cfg.initial_gear = gear;
            cfg.reverse_enabled = reverseEnabled;
            [h,distance,method] = vhybrid_heuristic( ...
                [0,0,0,2,3],[0,0,pi/2,0,20],cfg);

            testCase.verifyEqual(distance,5.5*pi/2,'AbsTol',1e-11);
            testCase.verifyEqual(h,3*5.5*pi/2,'AbsTol',1e-11);
            testCase.verifyEqual(method,'kinematic_lower_bound');
        end

        function straightOnePointTwoFiveMetresHasBoundedHeuristicChange(testCase)
            % 真实004父链中的直行边，旧双向Dubins在该边出现约91.4的跳变。
            cfg = testCase.lowerBoundConfig();
            before = [22.910133046786118,19.005428663389274, ...
                -1.7518181818115177,2,1];
            after = [22.68508951504402,17.77585336727234, ...
                -1.7518181818115177,3,1.5];
            goal = [10,8,-1.57,0,0];
            [hBefore,dBefore] = vhybrid_heuristic(before,goal,cfg);
            [hAfter,dAfter] = vhybrid_heuristic(after,goal,cfg);

            testCase.verifyEqual(hypot(after(1)-before(1), ...
                after(2)-before(2)),1.25,'AbsTol',1e-11);
            testCase.verifyLessThanOrEqual(abs(dAfter-dBefore),1.25+1e-11);
            testCase.verifyLessThanOrEqual(abs(hAfter-hBefore),3*1.25+1e-11);
            testCase.verifyLessThan(hAfter,hBefore);
        end

        function headingAcrossBranchCutRemainsContinuous(testCase)
            cfg = testCase.lowerBoundConfig();
            [hLeft,dLeft] = vhybrid_heuristic( ...
                [0,0,pi-1e-6,0,0],[0,0,0,0,0],cfg);
            [hRight,dRight] = vhybrid_heuristic( ...
                [0,0,-pi+1e-6,0,0],[0,0,0,0,0],cfg);

            testCase.verifyEqual(dLeft,dRight,'AbsTol',1e-11);
            testCase.verifyEqual(hLeft,hRight,'AbsTol',1e-11);
        end

        function changingWeightsUsesExistingProduct(testCase)
            cfg = testCase.lowerBoundConfig();
            cfg.heuristic_weight = 2.5;
            cfg.weight_goal_distance = 0.75;
            [h,distance] = vhybrid_heuristic([0,0,0,0,0],[3,4,0,0,0],cfg);

            testCase.verifyEqual(distance,5,'AbsTol',1e-12);
            testCase.verifyEqual(h,9.375,'AbsTol',1e-12);
        end

        function defaultPreservesLegacyEuclideanWithoutRadius(testCase)
            cfg = struct('weight_goal_distance',2);
            [h,distance,method] = vhybrid_heuristic( ...
                [0,0,0,0,0],[3,4,pi/2,0,0],cfg);

            testCase.verifyEqual(distance,5+pi/4,'AbsTol',1e-12);
            testCase.verifyEqual(h,2*(5+pi/4),'AbsTol',1e-12);
            testCase.verifyEqual(method,'euclidean');
        end

        function explicitEuclideanPreservesLegacyFormula(testCase)
            cfg = struct('heuristic_method','euclidean', ...
                'weight_goal_distance',2,'heuristic_weight',1.5);
            [h,distance,method] = vhybrid_heuristic( ...
                [0,0,0,0,0],[3,4,pi/2,0,0],cfg);

            testCase.verifyEqual(distance,5+pi/4,'AbsTol',1e-12);
            testCase.verifyEqual(h,3*(5+pi/4),'AbsTol',1e-12);
            testCase.verifyEqual(method,'euclidean');
        end

        function forwardDubinsPreservesKnownQuarterCircle(testCase)
            cfg = testCase.lowerBoundConfig();
            cfg.heuristic_method = 'dubins';
            cfg.reverse_enabled = false;
            [h,distance,method] = vhybrid_heuristic( ...
                [0,0,0,0,0],[5.5,5.5,pi/2,0,0],cfg);

            testCase.verifyEqual(distance,5.5*pi/2,'AbsTol',1e-9);
            testCase.verifyEqual(h,3*5.5*pi/2,'AbsTol',1e-9);
            testCase.verifyEqual(method,'dubins');
        end

        function reverseDubinsPreservesStraightConnectionBehindVehicle(testCase)
            cfg = testCase.lowerBoundConfig();
            cfg.heuristic_method = 'dubins';
            cfg.reverse_enabled = true;
            [h,distance,method] = vhybrid_heuristic( ...
                [0,0,0,0,0],[-10,0,0,0,0],cfg);

            testCase.verifyEqual(distance,10,'AbsTol',1e-9);
            testCase.verifyEqual(h,30,'AbsTol',1e-9);
            testCase.verifyEqual(method,'bidirectional_dubins_guidance');
        end

        function invalidRadiusHasExplicitError(testCase,invalidRadius)
            cfg = testCase.lowerBoundConfig();
            cfg.minimum_turning_radius_m = invalidRadius;

            testCase.verifyError(@() vhybrid_heuristic( ...
                [0,0,0,0,0],[3,4,0,0,0],cfg),'VHybrid:InvalidTurningRadius');
        end

        function missingRadiusHasExplicitError(testCase)
            cfg = struct('heuristic_method','kinematic_lower_bound', ...
                'weight_goal_distance',1);

            testCase.verifyError(@() vhybrid_heuristic( ...
                [0,0,0,0,0],[3,4,0,0,0],cfg),'VHybrid:MissingTurningRadius');
        end
    end

    methods (Static, Access = private)
        function cfg = lowerBoundConfig()
            cfg = struct('heuristic_method','kinematic_lower_bound', ...
                'minimum_turning_radius_m',5.5, ...
                'weight_goal_distance',2,'heuristic_weight',1.5);
        end
    end
end
