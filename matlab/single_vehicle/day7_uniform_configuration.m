function [scenario,planner] = day7_uniform_configuration(scenario,planner)
%day7_uniform_configuration - Validate a shared four-vehicle planning policy
%   SCENARIO = day7_uniform_configuration(SCENARIO,PLANNER) validates four
%   finite poses and one explicit planning order, and wraps headings to pi.
%   Vehicle labels only identify inputs, occupancy owners and output rows.
%
%   [SCENARIO,PLANNER] = day7_uniform_configuration(...) also returns the
%   common planner. Per-vehicle overrides and historical path inputs fail
%   explicitly. Dynamics, controls and precision remain configuration data.
%
%   See also run_four_vehicle_lateral_planning, summon_vhybrid_astar

if ~isstruct(planner) || ~isscalar(planner) || ~isfield(planner,'vhybrid') || ...
        ~isstruct(planner.vhybrid) || ~isscalar(planner.vhybrid)
    error('Day7Uniform:InvalidPlanner','Provide one scalar planner configuration.');
end
if ~isstruct(scenario) || ~isscalar(scenario)
    error('Day7Uniform:InvalidScenario','Provide one scalar scenario configuration.');
end
forbidden = {'day7_vehicle_search_overrides','vehicle_search_overrides', ...
    'initial_trajectory','initial_path','baseline_trajectory_file','warm_start'};
for k = 1:numel(forbidden)
    name = forbidden{k};
    if isfield(scenario,name) || isfield(planner,name) || isfield(planner.vhybrid,name)
        error('Day7Uniform:OverridesForbidden','Forbidden configuration field: %s.',name);
    end
end
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~isfield(scenario,'vehicles') || numel(scenario.vehicles)~=4 || ...
        ~isfield(scenario,'planning_order')
    error('Day7Uniform:InvalidScenario','Specify four vehicles and planning_order.');
end
vehicles = scenario.vehicles;
if ~isstruct(vehicles) || ~all(isfield(vehicles,{'id','start_pose','summon_goal'}))
    error('Day7Uniform:InvalidScenario','Each vehicle needs id, start_pose and summon_goal.');
end
ids = {vehicles.id};
order = scenario.planning_order;
if isstring(order), order = cellstr(order); end
if ~iscell(order) || numel(order)~=4 || ...
        ~all(cellfun(@validId,ids)) || ~all(cellfun(@validId,order))
    error('Day7Uniform:InvalidOrder','IDs and planning_order must contain four text labels.');
end
ids = cellfun(@char,ids,'UniformOutput',false);
order = cellfun(@char,order(:).','UniformOutput',false);
if numel(unique(ids))~=4 || numel(unique(order))~=4 || ~all(ismember(order,ids))
    error('Day7Uniform:InvalidOrder','planning_order must contain each unique vehicle exactly once.');
end
for k = 1:4
    for field = {'start_pose','summon_goal'}
        pose = vehicles(k).(field{1});
        if ~isstruct(pose) || ~isscalar(pose) || ...
                ~all(isfield(pose,{'x_m','y_m','yaw_rad'})) || ...
                ~finiteScalar(pose.x_m) || ~finiteScalar(pose.y_m) || ~finiteScalar(pose.yaw_rad)
            error('Day7Uniform:InvalidPose','%s has an invalid %s.',ids{k},field{1});
        end
        pose.yaw_rad = mod(double(pose.yaw_rad)+pi,2*pi)-pi;
        vehicles(k).(field{1}) = pose;
    end
    vehicles(k).id = ids{k};
    % There is one source of order; historical priority metadata is replaced.
    vehicles(k).priority = find(strcmp(order,ids{k}),1);
end
constraint = validate_lateral_acceleration_config(planner.dynamics);
if ~constraint.enabled || abs(constraint.max_lateral_acceleration_mps2-4.9)>1e-12
    error('Day7Uniform:InvalidLateralLimit','Enable the 4.9 m/s^2 hard lateral limit.');
end
required = {'heuristic_method','heuristic_weight','goal_connection_selection', ...
    'solution_refinement_time_s','goal_connection_duplicate_profile_pruning_enabled', ...
    'dynamic_braking_dominance_pruning_enabled'};
if ~all(isfield(planner.vhybrid,required))
    error('Day7Uniform:IncompletePolicy','Declare every shared search policy explicitly.');
end
if ~islogical(planner.vhybrid.dynamic_braking_dominance_pruning_enabled) || ...
        ~isscalar(planner.vhybrid.dynamic_braking_dominance_pruning_enabled)
    error('Day7Uniform:InvalidPruning','Declare the common braking pruning switch as logical.');
end
if planner.vhybrid.initial_speed_mps~=0 || planner.vhybrid.terminal_speed_mps~=0 || ...
        planner.day6.require_immediate_start || ~planner.day6.goal_hold_enabled
    error('Day7Uniform:InvalidBoundaryPolicy','Use rest at t=0, optional waiting and terminal parking.');
end
if ~finiteScalar(planner.vhybrid.max_search_nodes) || planner.vhybrid.max_search_nodes<2 || ...
        fix(planner.vhybrid.max_search_nodes)~=planner.vhybrid.max_search_nodes || ...
        ~finiteScalar(planner.vhybrid.max_search_time_s) || planner.vhybrid.max_search_time_s<=0
    error('Day7Uniform:InvalidBudget','Use a common positive time and integer node budget.');
end
scenario.vehicles = vehicles;
scenario.planning_order = order;
end

function tf = finiteScalar(value)
tf = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function tf = validId(value)
tf = (ischar(value) && isrow(value) && ~isempty(value)) || ...
    (isstring(value) && isscalar(value) && ~ismissing(value) && strlength(value)>0);
end
