# Day 8 Path Optimization Candidate

## Scope

Day 8 adds an independent geometry-only optimizer after a V-Hybrid A* rough path. It does not change `run_four_vehicle_lateral_planning`, `run_day7_lateral_limit_task4`, or the Day 7 verification and space-time occupancy registration. A Day 8 candidate is never inserted into the shared occupancy map and is never labeled executable.

The optimizer preserves endpoints, zero-speed stop anchors, and both sides of gear changes. It resamples each moving gear run by arc length, solves each run in its local corridor, and then performs independent static rectangle and steering-angle validation. Reverse runs are kept separate; their body curvature is the gear sign times course curvature.

## Paper Equations and Implementation

The reference is Section IV, equations (17)-(25), printed pages 12728-12729 of `Multi-Vehicle_Collaborative_Trajectory_Planning_in_Unstructured_Conflict_Areas_B.pdf`.

| Paper term | Implementation |
| --- | --- |
| (17), weighted objective | `vhybrid_optimize_path/solveRun` combines smoothness, reference fidelity, and spacing terms using the configured weights. |
| (19), smoothness `fs` | Sum of squared second differences of XY positions within each moving gear run. |
| (20), reference `fr` | Sum of squared XY offsets from the arc-length-resampled rough path. |
| (21), spacing `fl` | Convex quadratic approximation described below. |
| (22)-(23), corridor half-width | `vhybrid_build_path_corridor` applies the paper taper to `base_clearance_width_m/sqrt(2)`. |
| (24), endpoint index `mu` | Zero-based index distance to the nearer end of the current moving run, without normalization. |
| (25), contracted bounds | Nonnegative symmetric inward contraction of the paper box; fixed anchors have equal lower and upper bounds at the reference point. |

Equation (21) is ambiguous as printed: its displayed norm/sum does not explicitly subtract a target spacing, while the surrounding text says neighboring waypoint lengths should be uniform. The implementation uses

`fl = sum_k ||(p(k+1)-p(k)) - (p_ref(k+1)-p_ref(k))||^2`.

This is a convex quadratic penalty on deviations from the equal-arc-length reference step vectors. It is an approximation, not a literal reproduction of equation (21), and it encourages the optimized steps to retain their reference directions as well as their lengths.

For a moving run with `n` resampled points, define `N=n-1` and a zero-based local index `k=0,...,N`. The code uses `mu=min(k,N-k)`, which is the one-based MATLAB expression `min(localIndex-1,n-localIndex)`. The taper is `1/(1+exp(4-mu))`. The code does not normalize `mu` by run length.

For each reference pose, the corridor starts from the paper half-width `paper_half = base_clearance_width_m/sqrt(2) * taper`. The actual vehicle rectangle, including the configured front/rear/side collision margins, is checked against the map first. The minimum rectangle-to-boundary/obstacle gap is reduced by a rotation allowance `2*R*sin(heading_uncertainty_rad/2)` and `clearance_reserve_m`, where `R` is the conservative radius to the farthest vehicle corner. The safe half-width is the smaller of the paper half-width and `available_clearance/sqrt(2)`. Thus a world-axis-aligned square's maximum center displacement is bounded by the remaining clearance. Equation (25) is applied as `lower=reference-paper_half+epsilon` and `upper=reference+paper_half-epsilon`, with `epsilon=max(0,paper_half-safe_half)` on each coordinate. This is a local conservative corridor, not a replacement for vehicle collision validation.

The objective terms are divided by their number of contributing points/edges and by `resample_spacing_m^2`. Each raw term has units of square meters, so the normalized terms are dimensionless; the configured weights then express relative importance. This scaling keeps the weights comparable when the number of resampled points changes. The configuration's initial weights are `smoothness=1.0`, `reference=0.2`, and `spacing=0.5`. The separate `curvature_safety_factor=0.95` tightens only the QP's internal curvature envelope; the independent validator still checks the configured maximum steering angle itself.

The optimizer uses a sequential linearization of a discrete curvature bound while solving each run. The independent validator recomputes curvature from changes in segment course angle divided by the mean adjacent segment length. It copies the nearest interior curvature to each run endpoint to avoid an unstable one-sided endpoint estimate. For reverse travel, body curvature is `gear * course_curvature`; steering is `atan(wheelbase * body_curvature)`. The checked steering limit is the smaller of the existing vehicle and planner limits. The project vehicle dimensions, wheelbase, steering configuration, collision margins, map, and obstacles remain authoritative.

Static rectangle checking evaluates each path vertex and samples every adjacent segment, including segments across stop and gear-change anchors. The configured sampling resolution is 0.05 m in translation and 0.5 degrees in heading. This is a fixed-step sampled sweep, not a continuous analytical swept-volume proof; the report includes the method and resolutions. Candidates must pass this check and the maximum steering-angle check before `report.valid` is true.

Steering rate, speed/time parameterization, dynamic collision, and four-vehicle space-time conflict are marked `not_assessed_day9`. A passing result is only a static-safe geometric path candidate.

## Verification

On MATLAB R2023a with Optimization Toolbox 9.5, `tests/test_day8_path_optimization.m` passes all 9 tests. MATLAB Code Analyzer reports no issues in the three new functions or the test file.

A read-only smoke run used the four rough paths saved in the existing Day 7 result MAT file, in its recorded order. The candidates for `vehicle_001`, `vehicle_002`, and `vehicle_004` passed both static rectangle sweep and maximum steering-angle validation; each remains `is_executable=false`. `vehicle_003` returned `solver_failed_or_infeasible` (`quadprog` exit flag `-2`), so it was not accepted. Its untouched resampled rough geometry was statically clear but had a measured peak steering angle of 0.4697 rad against a 0.4680 rad limit; the local QP also reported a 0.0016 constraint violation. This saved-path smoke test is diagnostic only and does not substitute for a fresh Day 7 search or modify any Day 7 output.

## Configuration and Solver

Configuration: `config/day8_path_optimization.json`.

- `enabled`: explicit on/off switch for the standalone optimizer.
- `resample_spacing_m`: target spacing; each moving run is sampled uniformly with its final step no larger than this value.
- `corridor`: method, paper width, minimum feasible half-width, clearance reserve, heading uncertainty, taper switch, and minimum endpoint tangent projection.
- `weights`: objective weights, curvature safety factor, and sequential curvature-linearization iteration count.
- `solver`: `quadprog` algorithm, iteration budget, and tolerances.
- `validation`: rectangle pose-sweep translation/heading resolution and steering-angle tolerance.

The configured solver is Optimization Toolbox `quadprog` with the `interior-point-convex` algorithm. The initial curvature linearization count is 8: a read-only smoke test of the saved `vehicle_001` Day 7 rough result failed the independent turn-angle check at 3 iterations, was marginal at 5, and passed at 8 with margin. This changes only the Day 8 optimizer setting. Missing toolbox/solver support, unsupported solver names, solver failure, and infeasible corridors return explicit failed statuses; they do not return a passing candidate.

## Running

Run from MATLAB R2023a in the project root. An existing V-Hybrid A* result can be passed directly as `roughPath`; this does not rerun or alter the four-vehicle workflow.

```matlab
projectRoot = pwd;
addpath(fullfile(projectRoot, 'matlab', 'single_vehicle'));

vehicleConfig = jsondecode(fileread(fullfile(projectRoot, ...
    'config', 'vehicle_config_day7.json')));
plannerConfig = jsondecode(fileread(fullfile(projectRoot, ...
    'config', 'planner_config_four_vehicle_uniform.json')));
mapConfig = jsondecode(fileread(fullfile(projectRoot, ...
    'config', 'map_config_day7.json')));
optimizationConfig = jsondecode(fileread(fullfile(projectRoot, ...
    'config', 'day8_path_optimization.json')));
map = summon_map(mapConfig);

[candidate, report] = vhybrid_optimize_path(roughPath, map, ...
    vehicleConfig, plannerConfig, optimizationConfig);
```

Run the Day 8 tests with:

```matlab
results = runtests(fullfile(projectRoot, 'tests', ...
    'test_day8_path_optimization.m'));
assertSuccess(results);
```

The tests cover straight and curved paths, endpoint and stop/gear anchors, corridor shrinkage near an obstacle, sampled rectangle collisions (including across an anchor boundary), steering-angle rejection, paper taper/index behavior, invalid configuration, and explicit corridor/solver failure.
