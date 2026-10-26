classdef test_static_collision_context < matlab.unittest.TestCase
    %TEST_STATIC_COLLISION_CONTEXT 冻结静态内核、公共验证入口及缓存失效回归。
    % 已知几何覆盖接触/穿越；随机判据使用独立投影分离轴，而非角点算法。

    properties (TestParameter)
        boundaryCase = struct( ...
            'rearTouch',struct('pose',[0.5,5,0],'blocked',false,'reason','none'), ...
            'rearOutside',struct('pose',[0.5-1e-9,5,0],'blocked',true,'reason','road_boundary'), ...
            'frontTouch',struct('pose',[9,5,0],'blocked',false,'reason','none'), ...
            'frontOutside',struct('pose',[9+1e-9,5,0],'blocked',true,'reason','road_boundary'), ...
            'sideTouch',struct('pose',[5,0.2,0],'blocked',false,'reason','none'), ...
            'sideOutside',struct('pose',[5,0.2-1e-9,0],'blocked',true,'reason','road_boundary'), ...
            'rotatedInside',struct('pose',[5,5,pi/3],'blocked',false,'reason','none'), ...
            'rotatedOutside',struct('pose',[0.3,0.3,pi/4],'blocked',true,'reason','road_boundary'));
        obstacleCase = struct( ...
            'touch',struct('polygon',[6,4.9;6.2,4.9;6.2,5.1;6,5.1], ...
                'blocked',true,'reason','static_obstacle'), ...
            'gap',struct('polygon',[6+1e-9,4.9;6.2,4.9;6.2,5.1;6+1e-9,5.1], ...
                'blocked',false,'reason','none'), ...
            'edgeCross',struct('polygon',[5.2,4.5;5.3,4.5;5.3,5.5;5.2,5.5], ...
                'blocked',true,'reason','static_obstacle'), ...
            'obstacleInsideVehicle',struct('polygon',[5,4.9;5.1,4.9;5.1,5.1;5,5.1], ...
                'blocked',true,'reason','static_obstacle'), ...
            'vehicleInsideObstacle',struct('polygon',[4,4;7,4;7,6;4,6], ...
                'blocked',true,'reason','static_obstacle'));
        changedSource = {'dimensions','margin','boundary','obstacles'};
        invalidPose = {NaN(1,3),[1,2],complex([1,2,3],1),[1,2,Inf]};
    end

    methods (TestClassSetup)
        function addSource(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end

    methods (TestMethodSetup)
        function deterministicRandomSamples(testCase)
            savedRng = rng;
            testCase.addTeardown(@()rng(savedRng));
            rng(20261025,'twister');
        end
    end

    methods (Test)
        function boundaryContactAndRotation(testCase,boundaryCase)
            vehicle = testCase.smallVehicle();
            map = prepare_static_context(testCase.emptyMap(),vehicle);
            [public,detail] = day6_static_collision_check(boundaryCase.pose,map,vehicle);
            [core,coreDetail] = day6_static_collision_core(boundaryCase.pose,map.collision_context);
            testCase.verifyEqual(public,boundaryCase.blocked);
            testCase.verifyEqual(core,public);
            testCase.verifyEqual(day6_static_collision_core( ...
                boundaryCase.pose,map.collision_context),public);
            testCase.verifyEqual(day6_static_collision_check(boundaryCase.pose,map,vehicle),public);
            testCase.verifyEqual(detail.type,boundaryCase.reason);
            testCase.verifyEqual(detail.obstacle_index,0);
            testCase.verifySameDetail(coreDetail,detail);
        end

        function obstacleContactContainmentAndEdgeCrossing(testCase,obstacleCase)
            vehicle = testCase.smallVehicle();
            map = testCase.emptyMap();
            map.obstacles = {obstacleCase.polygon};
            map = prepare_static_context(map,vehicle);
            [public,detail] = day6_static_collision_check([5,5,0],map,vehicle);
            [core,coreDetail] = day6_static_collision_core([5,5,0],map.collision_context);
            testCase.verifyEqual(public,obstacleCase.blocked);
            testCase.verifyEqual(core,public);
            testCase.verifyEqual(detail.type,obstacleCase.reason);
            testCase.verifyEqual(detail.obstacle_index,double(obstacleCase.blocked));
            testCase.verifyEqual(detail.corners_xy, ...
                [6,5.2;6,4.8;4.5,4.8;4.5,5.2],'AbsTol',1e-12);
            testCase.verifySameDetail(coreDetail,detail);
        end

        function concaveBoundaryRejectsVehicleEdgeOutsideRoad(testCase)
            vehicle = testCase.smallVehicle();
            map = testCase.emptyMap();
            map.boundary_xy = [0,0;10,0;10,10;5.2,10;5.2,4;4.8,4;4.8,10;0,10];
            map = prepare_static_context(map,vehicle);
            [public,detail] = day6_static_collision_check([5,5,0],map,vehicle);
            corners = detail.corners_xy;
            % 四角都在道路内，缺口仅穿过车身边；角点检测本身无法发现。
            testCase.verifyTrue(all(inpolygon(corners(:,1),corners(:,2), ...
                map.boundary_xy(:,1),map.boundary_xy(:,2))));
            testCase.verifyTrue(public);
            testCase.verifyTrue(day6_static_collision_core([5,5,0],map.collision_context));
            testCase.verifyEqual(detail.type,'road_boundary');
        end

        function convexNonRectangleBoundaryUsesPolygonGeometry(testCase)
            vehicle = testCase.smallVehicle();
            map = testCase.emptyMap();
            map.boundary_xy = [0,5;5,0;10,5;5,10];
            map = prepare_static_context(map,vehicle);
            flags = testCase.comparePublicAndCore([5,5,pi/4;0.6,5,0],map,vehicle);
            testCase.verifyFalse(map.collision_context.is_rectangle);
            testCase.verifyEqual(flags,repmat([false;true],1,2));
        end

        function randomPosesAgreeWithIndependentProjectionOracle(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.collision_margin_m = struct('front',0.13,'rear',0.07,'side',0.11);
            map = testCase.emptyMap();
            map.obstacles = {[2,2;3.5,2;3.5,4;2,4], ...
                [6,5;8,5;8,7;6,7],[4,6;5,9;3.5,8]};
            poses = [12*rand(700,2)-1,2*pi*rand(700,1)-pi];
            actual = testCase.comparePublicAndCore(poses,map,vehicle);
            expected = testCase.projectionOracle(poses,map,vehicle);
            testCase.verifyEqual(actual,repmat(expected,1,2));
            testCase.verifyTrue(any(expected));
            testCase.verifyTrue(any(~expected));
        end

        function sourceMutationInvalidatesCachedGeometry(testCase,changedSource)
            vehicle = testCase.smallVehicle();
            cached = prepare_static_context(testCase.emptyMap(),vehicle);
            before = day6_static_collision_check([5,5,0],cached,vehicle);
            [map,vehicle] = testCase.mutateSource(cached,vehicle,changedSource);
            fromCached = day6_static_collision_check([5,5,0],map,vehicle);
            refreshed = prepare_static_context(map,vehicle);
            fresh = rmfield(map,'collision_context');
            testCase.verifyFalse(before);
            testCase.verifyTrue(fromCached);
            testCase.verifyEqual(fromCached,day6_static_collision_check([5,5,0],fresh,vehicle));
            testCase.verifyEqual(fromCached, ...
                day6_static_collision_core([5,5,0],refreshed.collision_context));
            testCase.verifyEqual(refreshed.collision_context.dimensions_m,vehicle.dimensions_m);
            testCase.verifyEqual(refreshed.collision_context.margin,vehicle.collision_margin_m);
            testCase.verifyEqual(refreshed.collision_context.source_boundary,map.boundary_xy);
            testCase.verifyEqual(refreshed.collision_context.source_obstacles,map.obstacles);
        end

        function unchangedPreparationIsIdempotent(testCase)
            vehicle = testCase.smallVehicle();
            map = prepare_static_context(testCase.emptyMap(),vehicle);
            again = prepare_static_context(map,vehicle);
            testCase.verifyEqual(again,map);
        end

        function missingMarginsRetainZeroDefault(testCase)
            vehicle = rmfield(testCase.smallVehicle(),'collision_margin_m');
            map = prepare_static_context(testCase.emptyMap(),vehicle);
            testCase.verifyFalse(day6_static_collision_check([0.5,5,0],map,vehicle));
            testCase.verifyEqual(map.collision_context.margin,struct('front',0,'rear',0,'side',0));
        end

        function publicEntryStillValidatesPose(testCase,invalidPose)
            testCase.verifyError(@()day6_static_collision_check( ...
                invalidPose,testCase.emptyMap(),testCase.smallVehicle()), ...
                'Day6:StaticCollision:InvalidPose');
        end

        function invalidChangedBoundaryIsRevalidated(testCase)
            vehicle = testCase.smallVehicle();
            map = prepare_static_context(testCase.emptyMap(),vehicle);
            map.boundary_xy(1,1) = NaN;
            testCase.verifyError(@()day6_static_collision_check([5,5,0],map,vehicle), ...
                'Day6:StaticCollision:InvalidPolygon');
        end
    end

    methods (Access=private)
        function verifySameDetail(testCase,actual,expected)
            testCase.verifyEqual(actual.type,expected.type);
            testCase.verifyEqual(actual.obstacle_index,expected.obstacle_index);
            testCase.verifyEqual(actual.corners_xy,expected.corners_xy,'AbsTol',1e-12);
        end
    end

    methods (Static,Access=private)
        function vehicle = smallVehicle()
            vehicle = struct('dimensions_m',struct('width',0.4,'wheelbase',0.6, ...
                'front_axle_to_front',0.4,'rear_axle_to_rear',0.5), ...
                'collision_margin_m',struct('front',0,'rear',0,'side',0));
        end

        function map = emptyMap()
            map = struct('boundary_xy',[0,0;10,0;10,10;0,10],'obstacles',{{}});
        end

        function flags = comparePublicAndCore(poses,map,vehicle)
            prepared = prepare_static_context(map,vehicle);
            flags = false(size(poses,1),2);
            for k = 1:size(poses,1)
                flags(k,1) = day6_static_collision_check(poses(k,:),map,vehicle);
                flags(k,2) = day6_static_collision_core(poses(k,:),prepared.collision_context);
            end
        end

        function flags = projectionOracle(poses,map,vehicle)
            % 独立解析支持函数：不生成车角点、不调用inpolygon或线段求交。
            d = vehicle.dimensions_m; margin = vehicle.collision_margin_m;
            front = d.wheelbase+d.front_axle_to_front+margin.front;
            rear = d.rear_axle_to_rear+margin.rear;
            halfLength = (front+rear)/2; halfWidth = d.width/2+margin.side;
            lower = min(map.boundary_xy,[],1); upper = max(map.boundary_xy,[],1);
            flags = false(size(poses,1),1);
            for k = 1:size(poses,1)
                heading = [cos(poses(k,3)),sin(poses(k,3))];
                side = [-heading(2),heading(1)];
                center = poses(k,1:2)+(front-rear)/2*heading;
                extent = halfLength*abs(heading)+halfWidth*abs(side);
                flags(k) = any(center-extent<lower | center+extent>upper);
                for obstacleId = 1:numel(map.obstacles)
                    polygon = map.obstacles{obstacleId};
                    edges = polygon([2:end,1],:)-polygon;
                    axes = [heading;side;-edges(:,2),edges(:,1)];
                    centerProjection = center*axes.';
                    radius = halfLength*abs(heading*axes.')+halfWidth*abs(side*axes.');
                    obstacleProjection = polygon*axes.';
                    overlap = all(centerProjection+radius>=min(obstacleProjection,[],1) & ...
                        centerProjection-radius<=max(obstacleProjection,[],1));
                    flags(k) = flags(k) || overlap;
                end
            end
        end

        function [map,vehicle] = mutateSource(map,vehicle,source)
            switch source
                case 'dimensions'
                    vehicle.dimensions_m.width = 12;
                case 'margin'
                    vehicle.collision_margin_m.side = 5;
                case 'boundary'
                    map.boundary_xy = [0,0;4,0;4,10;0,10];
                case 'obstacles'
                    map.obstacles = {[5,4.9;5.1,4.9;5.1,5.1;5,5.1]};
            end
        end
    end
end
