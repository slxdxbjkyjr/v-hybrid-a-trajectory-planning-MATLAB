function [h,distance,method] = vhybrid_heuristic(state,goal,cfg)
%VHYBRID_HEURISTIC 起点和扩展节点共用的目标启发代价。
% 输入：连续状态[x,y,theta,v,t]、目标状态和vhybrid集中配置。
% 输出：加权启发h、未加权几何距离distance和实际采用的方法method。
% Day4默认保留欧氏距离加航向项。Day6选择dubins时使用满足最小转弯
% 半径的无障碍前向最短Dubins弧长。启用倒车后取前向/完整倒车两种
% 弧长的较小值作为搜索引导；这不是包含混合换挡的 Reeds-Shepp 下界。
% 权重大于1的加权A*及此双向引导不保证全局最优。
% 可选kinematic_lower_bound取欧氏距离与R*最小航向差的较大值，
% 作为允许换挡、曲率受限轨迹的几何长度下界，避免单挡Dubins拓扑跳变。
% 几何下界不等于完整加权搜索代价的可采纳启发，也不保证有限预算最优。
method = 'euclidean';
if isfield(cfg,'heuristic_method'), method = char(cfg.heuristic_method); end
if strcmp(method,'dubins')
    if ~isfield(cfg,'minimum_turning_radius_m')
        error('VHybrid:MissingTurningRadius','Dubins启发需要由车辆模型计算的最小转弯半径。');
    end
    curves = vhybrid_dubins_candidates(state(1:3),goal(1:3),cfg.minimum_turning_radius_m);
    if isempty(curves)
        distance = hypot(state(1)-goal(1),state(2)-goal(2));
    else
        distance = sum(curves(1).lengths);
    end
    if isfield(cfg,'reverse_enabled') && cfg.reverse_enabled
        reverse_state = state(1:3); reverse_goal = goal(1:3);
        reverse_state(3) = reverse_state(3)+pi; reverse_goal(3) = reverse_goal(3)+pi;
        reverse_curves = vhybrid_dubins_candidates(reverse_state,reverse_goal,cfg.minimum_turning_radius_m);
        if ~isempty(reverse_curves), distance = min(distance,sum(reverse_curves(1).lengths)); end
        method = 'bidirectional_dubins_guidance';
    end
elseif strcmp(method,'euclidean')
    distance = hypot(state(1)-goal(1),state(2)-goal(2)) + ...
        0.5*abs(mod(state(3)-goal(3)+pi,2*pi)-pi);
elseif strcmp(method,'kinematic_lower_bound')
    if ~isfield(cfg,'minimum_turning_radius_m')
        error('VHybrid:MissingTurningRadius','运动学距离下界需要由车辆模型计算的最小转弯半径。');
    end
    radius = cfg.minimum_turning_radius_m;
    if ~isnumeric(radius) || ~isscalar(radius) || ~isreal(radius) || ...
            ~isfinite(radius) || radius <= 0
        error('VHybrid:InvalidTurningRadius','最小转弯半径必须为正有限实数标量。');
    end
    position_distance = hypot(state(1)-goal(1),state(2)-goal(2));
    heading_difference = abs(mod(state(3)-goal(3)+pi,2*pi)-pi);
    % 两项各自约束总弧长；相加会重复计算，取max保持几何下界。
    distance = max(position_distance,radius*heading_difference);
else
    error('VHybrid:InvalidHeuristicMethod','未知启发式方法%s。',method);
end
weight = 1;
if isfield(cfg,'heuristic_weight'), weight = cfg.heuristic_weight; end
if ~isnumeric(weight) || ~isscalar(weight) || ~isfinite(weight) || weight<=0
    error('VHybrid:InvalidHeuristicWeight','启发式权重必须为正有限标量。');
end
h = weight*cfg.weight_goal_distance*distance;
end
