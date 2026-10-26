classdef test_vhybrid_speed_phases < matlab.unittest.TestCase
    %TEST_VHYBRID_SPEED_PHASES 验证连接器峰/谷及速度样本语义。

    properties
        Dynamics
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectDir,'matlab','single_vehicle')));
            testCase.Dynamics = struct('a_max_mps2',2,'a_min_mps2',-2, ...
                'v_min_mps',0,'v_max_mps',5);
        end
    end

    methods (Test)
        function lowerCruiseSampleCanDecelerateFromFasterParent(testCase)
            % 候选巡航速度1m/s低于父节点3m/s，仍可制动、巡航并停到0。
            [phases,feasible] = vhybrid_speed_phases(3.25,3,0,1,testCase.Dynamics);
            testCase.verifyTrue(feasible);
            testCase.verifyEqual(phases(:,1),[1;1;0.5],'AbsTol',1e-12);
            testCase.verifyEqual(phases(:,2),[-2;0;-2],'AbsTol',1e-12);
            testCase.verifyProfile(phases,3,0,3.25);
        end

        function tooShortToStopIsRejected(testCase)
            % 3m/s在最大减速度2m/s^2下至少需要2.25m才能停稳。
            [phases,feasible] = vhybrid_speed_phases(2.24,3,0,1,testCase.Dynamics);
            testCase.verifyFalse(feasible);
            testCase.verifyEmpty(phases);
        end

        function shortDistanceUsesTriangularPeakBelowSample(testCase)
            [phases,feasible] = vhybrid_speed_phases(1,0,0,2,testCase.Dynamics);
            testCase.verifyTrue(feasible);
            expected_peak = sqrt(2);
            testCase.verifyEqual(phases(:,3),[expected_peak;0],'AbsTol',1e-12);
            testCase.verifyProfile(phases,0,0,1);
        end

        function shortDistanceUsesValleyAboveSample(testCase)
            [phases,feasible] = vhybrid_speed_phases(2.125,3,2,1,testCase.Dynamics);
            testCase.verifyTrue(feasible);
            testCase.verifyEqual(phases(1,3),1.5,'AbsTol',1e-12);
            testCase.verifyEqual(phases(2,3),2,'AbsTol',1e-12);
            testCase.verifyProfile(phases,3,2,2.125);
        end

        function exactTerminalBrakingDistanceIsFeasible(testCase)
            [phases,feasible] = vhybrid_speed_phases(2.25,3,0,1,testCase.Dynamics);
            testCase.verifyTrue(feasible);
            testCase.verifyProfile(phases,3,0,2.25);
        end
    end

    methods (Access=private)
        function verifyProfile(testCase,phases,startSpeed,endSpeed,totalDistance)
            speed = startSpeed;
            distance = 0;
            for k = 1:size(phases,1)
                dt = phases(k,1);
                acceleration = phases(k,2);
                nextSpeed = speed+acceleration*dt;
                testCase.verifyGreaterThan(dt,0);
                testCase.verifyGreaterThanOrEqual(acceleration, ...
                    testCase.Dynamics.a_min_mps2-1e-12);
                testCase.verifyLessThanOrEqual(acceleration, ...
                    testCase.Dynamics.a_max_mps2+1e-12);
                testCase.verifyGreaterThanOrEqual(min(speed,nextSpeed), ...
                    testCase.Dynamics.v_min_mps-1e-12);
                testCase.verifyLessThanOrEqual(max(speed,nextSpeed), ...
                    testCase.Dynamics.v_max_mps+1e-12);
                testCase.verifyEqual(nextSpeed,phases(k,3),'AbsTol',1e-10);
                distance = distance+speed*dt+0.5*acceleration*dt^2;
                speed = nextSpeed;
            end
            testCase.verifyEqual(speed,endSpeed,'AbsTol',1e-10);
            testCase.verifyEqual(distance,totalDistance,'AbsTol',1e-9);
        end
    end
end
