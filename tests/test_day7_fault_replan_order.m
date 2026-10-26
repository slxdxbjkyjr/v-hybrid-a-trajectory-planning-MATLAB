classdef test_day7_fault_replan_order < matlab.unittest.TestCase
    %TEST_DAY7_FAULT_REPLAN_ORDER Day7故障影响车辆的稳定降级优先级测试。
    methods (TestClassSetup)
        function addDay7Functions(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function affectedCarsMoveAfterUnaffected(testCase)
            cars=testCase.fourCars();
            [ids,priorities]=day7_fault_replan_order(cars,{'vehicle_002','vehicle_003'});
            testCase.verifyEqual(ids,{'vehicle_001','vehicle_004','vehicle_002','vehicle_003'});
            testCase.verifyEqual(priorities,[1,5,6,4]);
        end

        function allCarsAffectedKeepTheirOldRelativeOrder(testCase)
            cars=testCase.fourCars();
            [ids,priorities]=day7_fault_replan_order(cars,{'vehicle_001','vehicle_002', ...
                'vehicle_003','vehicle_004'});
            testCase.verifyEqual(ids,{'vehicle_001','vehicle_002','vehicle_003','vehicle_004'});
            testCase.verifyEqual(priorities,[5,6,7,8]);
        end

        function unknownAffectedVehicleIsRejected(testCase)
            testCase.verifyError(@()day7_fault_replan_order(testCase.fourCars(),{'vehicle_099'}), ...
                'Day7:InvalidAffectedVehicles');
        end

        function duplicateAffectedVehicleIsRejected(testCase)
            testCase.verifyError(@()day7_fault_replan_order(testCase.fourCars(), ...
                {'vehicle_002','vehicle_002'}),'Day7:InvalidAffectedVehicles');
        end

        function malformedFaultEventStopsBeforeSearch(testCase)
            % 缺少真实停车前缀时必须明确拒绝，不能把普通输入错误当成故障重排。
            result = run_day7_sequential_planning_demo(false, ...
                struct('fault_event',struct('cause','test')));
            testCase.verifyFalse(result.valid);
            testCase.verifyEqual(result.reason,'invalid_fault_event');
            testCase.verifyTrue(isfield(result.fault_event_summary,'valid'));
            testCase.verifyFalse(result.fault_event_summary.valid);
        end
    end

    methods (Static,Access=private)
        function cars=fourCars()
            cars=repmat(struct('id','','priority',0),4,1);
            for k=1:4,cars(k).id=sprintf('vehicle_%03d',k);cars(k).priority=k;end
        end
    end
end
