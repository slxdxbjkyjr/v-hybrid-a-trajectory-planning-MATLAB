function [is_collision, detail] = day6_static_collision_check(pose,map,vehicle_config)
%DAY6_STATIC_COLLISION_CHECK 完整车辆矩形与原始静态多边形的适配检测。
% 输入：以后轴中心为参考的[x,y,theta]，summon_map几何和车辆尺寸/安全裕度。
% 输出：碰撞标志；detail.type/obstacle_index/corners_xy描述原因与车辆外包络。
% 逻辑：只对车辆矩形加一次配置裕度，不重复使用Day5全航向膨胀地图。
% 障碍物检测包含角点落入、障碍物被车身包住和边相交。凹道路边界除
% 检查角点外，还将车身边按所有边界交点分段，检查每个开段是否在道路内。
if ~isnumeric(pose) || numel(pose) ~= 3 || ~isreal(pose) || any(~isfinite(pose))
    error('Day6:StaticCollision:InvalidPose','位姿必须为有限实数[x,y,theta]。');
end
pose = double(pose(:).');
% 公共入口始终核对尺寸、裕度、边界及障碍物，修改源数据后重建缓存。
map = prepare_static_context(map,vehicle_config);
if nargout > 1
    [is_collision,detail] = day6_static_collision_core(pose,map.collision_context);
else
    is_collision = day6_static_collision_core(pose,map.collision_context);
end
end
