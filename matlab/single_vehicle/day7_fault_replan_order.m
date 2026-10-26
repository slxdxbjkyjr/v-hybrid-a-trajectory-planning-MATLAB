function [ordered_ids, assigned_priorities] = day7_fault_replan_order(vehicles, affected_ids)
%DAY7_FAULT_REPLAN_ORDER 将故障停车车辆移至队列末尾并保持原相对顺序。
% 输入：场景车辆结构数组和受同一故障影响、已安全停车的车辆ID。
% 输出：重规划ID顺序及每辆车故障后的有效优先级。
% 规则：未受影响车辆保留原优先级；受影响车辆按故障前优先级稳定排序，
%      依次分配比所有未受影响车辆更低的优先级（数值更大）。
if ~isstruct(vehicles) || isempty(vehicles) || ~all(isfield(vehicles,{'id','priority'}))
    error('Day7:InvalidVehicles','vehicles需要包含id和priority字段。');
end
ids = {vehicles.id};
priorities = double([vehicles.priority]);
if numel(unique(ids)) ~= numel(ids) || any(~isfinite(priorities)) || ...
        numel(unique(priorities)) ~= numel(priorities)
    error('Day7:InvalidVehicles','车辆ID和原优先级必须唯一且有效。');
end
if isstring(affected_ids), affected_ids = cellstr(affected_ids(:)); end
if ischar(affected_ids), affected_ids = {affected_ids}; end
if ~iscellstr(affected_ids) || isempty(affected_ids) || ... %#ok<ISCLSTR>
        numel(unique(affected_ids)) ~= numel(affected_ids) || any(~ismember(affected_ids,ids))
    error('Day7:InvalidAffectedVehicles','受影响车辆ID必须非空、唯一且属于当前场景。');
end
[~,base_order] = sort(priorities,'ascend');
unaffected = base_order(~ismember(ids(base_order),affected_ids));
affected = base_order(ismember(ids(base_order),affected_ids));
ordered_indices = [unaffected,affected];
ordered_ids = ids(ordered_indices);
assigned_priorities = priorities;
next_priority = max(priorities(unaffected),[],'omitnan');
if isempty(unaffected), next_priority = max(priorities); end
for k = 1:numel(affected)
    next_priority = next_priority+1;
    assigned_priorities(affected(k)) = next_priority;
end
end
