classdef test_day8_path_optimization < matlab.unittest.TestCase
    %TEST_DAY8_PATH_OPTIMIZATION Verify isolated geometric path candidates.
    % Candidates are checked as static geometry and are never executable.

    properties
        Project
        Map
        Vehicle
        Planner
        Optimization
    end

    methods (TestClassSetup)
        function addSourceAndLoadConfiguration(testCase)
            testCase.Project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.Project,'matlab','single_vehicle')));
            testCase.Vehicle = testCase.readConfig('vehicle_config_day7.json');
            testCase.Planner = testCase.readConfig('planner_config_four_vehicle_uniform.json');
            testCase.Map = summon_map(testCase.readConfig('map_config_day7.json'));
            testCase.Optimization = testCase.readConfig('day8_path_optimization.json');
        end
    end

    methods (Test)
        function straightPathPreservesEndpointsAndDoesNotDrift(testCase)
            x = (5:1:15).';
            path = testCase.makePath(x,15+zeros(size(x)),zeros(size(x)), ...
                ones(size(x)),2*ones(size(x)),false(size(x)));

            [candidate,report] = vhybrid_optimize_path(path,testCase.Map, ...
                testCase.Vehicle,testCase.Planner,testCase.Optimization);

            testCase.verifyTrue(report.valid,report.reason);
            testCase.verifyFalse(candidate.is_executable);
            testCase.verifyEqual([candidate.x(1),candidate.y(1)], ...
                [path.x(1),path.y(1)],'AbsTol',1e-10);
            testCase.verifyEqual([candidate.x(end),candidate.y(end)], ...
                [path.x(end),path.y(end)],'AbsTol',1e-10);
            testCase.verifyEqual(candidate.y,15*ones(size(candidate.y)),'AbsTol',2e-7);
            testCase.verifyTrue(report.static_collision_free);
            testCase.verifyTrue(report.steering_within_limit);
            testCase.verifyEqual(report.turning_rate_status,'not_assessed_day9');
            testCase.verifyEqual(report.spatiotemporal_collision_status,'not_assessed_day9');
        end

        function curvedPathReducesSecondDifference(testCase)
            x = (5:0.5:15).';
            y = 15+sin(pi*(x-5)/10);
            dy = (pi/10)*cos(pi*(x-5)/10);
            theta = atan(dy);
            path = testCase.makePath(x,y,theta,ones(size(x)), ...
                2*ones(size(x)),false(size(x)));

            [candidate,report] = vhybrid_optimize_path(path,testCase.Map, ...
                testCase.Vehicle,testCase.Planner,testCase.Optimization);

            testCase.verifyTrue(report.valid,report.reason);
            testCase.verifyLessThan(report.smoothness_after,report.smoothness_before);
            testCase.verifyEqual([candidate.x(1),candidate.y(1)], ...
                [path.x(1),path.y(1)],'AbsTol',1e-10);
            testCase.verifyEqual([candidate.x(end),candidate.y(end)], ...
                [path.x(end),path.y(end)],'AbsTol',1e-10);
            testCase.verifyTrue(report.steering_within_limit);
        end

        function narrowObstacleShrinksCorridorAndRectangleSweepRejectsCollision(testCase)
            map = testCase.Map;
            map.obstacles = {[10,17;10.5,17;10.5,20;10,20;10,17]};
            x = (5:0.5:15).';
            path = testCase.makePath(x,15+zeros(size(x)),zeros(size(x)), ...
                ones(size(x)),2*ones(size(x)),false(size(x)));
            reference = testCase.referenceFromPath(path);
            [corridor,corridorReport] = vhybrid_build_path_corridor( ...
                reference,map,testCase.Vehicle,testCase.Optimization);

            testCase.verifyTrue(corridorReport.valid,corridorReport.reason);
            testCase.verifyTrue(all(corridor.lower<=reference.xy+1e-12,'all'));
            testCase.verifyTrue(all(corridor.upper>=reference.xy-1e-12,'all'));
            testCase.verifyEqual(corridorReport.points(2).mu_index,1);
            expectedTaper = 1/(1+exp(4-1));
            testCase.verifyEqual(corridorReport.points(2).paper_half_width_m, ...
                testCase.Optimization.corridor.base_clearance_width_m/ ...
                sqrt(2)*expectedTaper,'AbsTol',1e-12);
            free_points = ~reference.anchor_mask;
            testCase.verifyLessThan(min(corridor.half_width(free_points)), ...
                testCase.Optimization.corridor.base_clearance_width_m/sqrt(2));

            unsafe = testCase.geometryCandidate((5:0.1:15).', ...
                18*ones(101,1),zeros(101,1),ones(101,1));
            validation = vhybrid_validate_path_geometry(unsafe,map, ...
                testCase.Vehicle,testCase.Planner,testCase.Optimization);
            testCase.verifyFalse(validation.static_collision_free);
            testCase.verifyTrue(contains(validation.reason,'collision'));
        end

        function maximumSteeringAngleIsEnforced(testCase)
            angle = linspace(0,pi/3,80).';
            radius = 4;
            x = 12+radius*cos(angle);
            y = 12+radius*sin(angle);
            theta = angle+pi/2;
            candidate = testCase.geometryCandidate(x,y,theta,ones(size(x)));

            validation = vhybrid_validate_path_geometry(candidate,testCase.Map, ...
                testCase.Vehicle,testCase.Planner,testCase.Optimization);

            testCase.verifyGreaterThan(validation.max_steer_angle_rad, ...
                min(testCase.Vehicle.steering.maximum_steer_rad, ...
                testCase.Planner.dynamics.delta_max_rad));
            testCase.verifyFalse(validation.steering_within_limit);
        end

        function stopsAndGearSwitchAnchorsAreRetained(testCase)
            x = [5;6;7;8;8;8;7;6;5];
            y = 12+zeros(size(x));
            theta = zeros(size(x));
            gear = [1;1;1;1;1;-1;-1;-1;-1];
            speed = [0;1;1;0;0;0;1;1;0];
            changed = false(size(x)); changed(6) = true;
            path = testCase.makePath(x,y,theta,gear,speed,changed);

            [candidate,report] = vhybrid_optimize_path(path,testCase.Map, ...
                testCase.Vehicle,testCase.Planner,testCase.Optimization);

            testCase.verifyTrue(report.valid,report.reason);
            testCase.verifyEqual([candidate.x(1),candidate.y(1)], ...
                [x(1),y(1)],'AbsTol',1e-10);
            testCase.verifyEqual([candidate.x(end),candidate.y(end)], ...
                [x(end),y(end)],'AbsTol',1e-10);
            atStop = abs(candidate.x-8)<1e-10 & abs(candidate.y-12)<1e-10;
            testCase.verifyTrue(any(atStop & candidate.gear==1));
            testCase.verifyTrue(any(atStop & candidate.gear==-1));
            testCase.verifyTrue(any(atStop & candidate.anchor_mask));
            testCase.verifyTrue(any(candidate.gear==-1));
            testCase.verifyFalse(candidate.is_executable);
            testCase.verifyFalse(candidate.time_parameterized);
        end

        function infeasibleCorridorReturnsExplicitFailure(testCase)
            map = testCase.Map;
            x = (5:1:12).';
            path = testCase.makePath(x,1.5+zeros(size(x)),zeros(size(x)), ...
                ones(size(x)),2*ones(size(x)),false(size(x)));
            config = testCase.Optimization;
            config.corridor.minimum_half_width_m = 0.5;

            [candidate,report] = vhybrid_optimize_path(path,map, ...
                testCase.Vehicle,testCase.Planner,config);

            testCase.verifyFalse(report.valid);
            testCase.verifyNotEqual(report.status,'passed');
            testCase.verifyTrue(contains(report.reason,'corridor'));
            testCase.verifyFalse(report.static_collision_free);
            testCase.verifyTrue(isempty(candidate) || ~candidate.is_executable);
        end

        function collisionAcrossAnchorBoundaryIsSwept(testCase)
            map = testCase.Map;
            map.obstacles = {[9.8,14;10.2,14;10.2,16;9.8,16;9.8,14]};
            candidate = testCase.geometryCandidate([5;15],[15;15],[0;0],[1;1]);
            candidate.run_id = [1;0];

            validation = vhybrid_validate_path_geometry(candidate,map, ...
                testCase.Vehicle,testCase.Planner,testCase.Optimization);

            testCase.verifyFalse(validation.static_collision_free);
            testCase.verifyGreaterThan(validation.swept_segment_count,0);
            testCase.verifyEqual(validation.first_collision.segment_index,1);
        end

        function unsupportedSolverReturnsExplicitFailure(testCase)
            x = (5:1:15).';
            path = testCase.makePath(x,15+zeros(size(x)),zeros(size(x)), ...
                ones(size(x)),2*ones(size(x)),false(size(x)));
            config = testCase.Optimization;
            config.solver.name = 'unsupported_solver';

            [candidate,report] = vhybrid_optimize_path(path,testCase.Map, ...
                testCase.Vehicle,testCase.Planner,config);

            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.reason,'unsupported_solver');
            testCase.verifyFalse(candidate.is_executable);
        end

        function invalidCorridorConfigurationIsRejected(testCase)
            x = (5:1:15).';
            path = testCase.makePath(x,15+zeros(size(x)),zeros(size(x)), ...
                ones(size(x)),2*ones(size(x)),false(size(x)));
            config = testCase.Optimization;
            config.corridor.minimum_neighbor_projection_m = 'invalid';

            testCase.verifyError(@()vhybrid_optimize_path(path,testCase.Map, ...
                testCase.Vehicle,testCase.Planner,config), ...
                'VHybridPath:InvalidConfig');
        end
    end

    methods (Access=private)
        function config = readConfig(testCase,name)
            config = jsondecode(fileread(fullfile(testCase.Project,'config',name)));
        end

        function path = makePath(~,x,y,theta,gear,speed,gearChange)
            x = double(x(:)); y = double(y(:)); theta = double(theta(:));
            gear = double(gear(:)); speed = double(speed(:));
            gearChange = logical(gearChange(:));
            n = numel(x);
            path = struct('x',x,'y',y,'theta',theta,'v',speed, ...
                't',(0:n-1).','acceleration',zeros(n,1), ...
                'steering_angle',zeros(n,1),'gear',gear, ...
                'is_gear_change',gearChange,'travelled_distance', ...
                [0;gear(2:end).*hypot(diff(x),diff(y))], ...
                'direction',[0;sign(gear(2:end).*hypot(diff(x),diff(y)))]);
        end

        function reference = referenceFromPath(~,path)
            n = numel(path.x);
            reference = struct('xy',[path.x,path.y],'x',path.x,'y',path.y, ...
                'theta',path.theta,'reference_theta',path.theta,'gear',path.gear, ...
                'run_id',ones(n,1),'anchor_mask',false(n,1));
            reference.anchor_mask([1,end]) = true;
        end

        function candidate = geometryCandidate(~,x,y,theta,gear)
            candidate = struct('x',double(x(:)),'y',double(y(:)), ...
                'theta',double(theta(:)),'gear',double(gear(:)), ...
                'run_id',ones(numel(x),1),'anchor_mask',false(numel(x),1), ...
                'reference_xy',[double(x(:)),double(y(:))], ...
                'reference_theta',double(theta(:)), ...
                'is_executable',false,'time_parameterized',false);
            candidate.anchor_mask([1,end]) = true;
        end
    end
end
