[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$SourceRoot,
    [string]$OutputDirectory = ''
)

# Read-only diagnostic of saved legacy CSV samples. This does not execute MATLAB,
# run a planner, retime a path, or accept a new constrained search trajectory.
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$sourcePath = (Resolve-Path -LiteralPath $SourceRoot).Path.TrimEnd('\', '/')
$stageRoot = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $stageRoot 'outputs\day7_lateral_constraint\task1\baseline_diagnostic_csv'
}
$outputPath = [System.IO.Path]::GetFullPath($OutputDirectory).TrimEnd('\', '/')
$sourceOutputPrefix = Join-Path $sourcePath 'outputs\day7_lateral_constraint'
$insideSource = $outputPath.Equals($sourcePath, [StringComparison]::OrdinalIgnoreCase) -or
    $outputPath.StartsWith($sourcePath + [System.IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)
$insideIsolatedOutput = $outputPath.Equals($sourceOutputPrefix, [StringComparison]::OrdinalIgnoreCase) -or
    $outputPath.StartsWith($sourceOutputPrefix + [System.IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)
if ($insideSource -and -not $insideIsolatedOutput) {
    throw 'An output inside SourceRoot must use outputs/day7_lateral_constraint; legacy configuration and output directories are read-only.'
}

$csvRelative = 'outputs\day7_four_vehicle\day7_trajectory_samples.csv'
$vehicleRelative = 'config\vehicle_config_day7.json'
$inputNames = @(
    $csvRelative, $vehicleRelative,
    'config\planner_config_day7.json',
    'config\scenario_day7_four_vehicle.json',
    'outputs\day7_four_vehicle\day7_four_vehicle_report.json',
    'outputs\day7_four_vehicle\day7_four_vehicle_trajectories.mat'
)
function Get-InputHashes {
    $records = foreach ($name in $inputNames) {
        $file = Join-Path $sourcePath $name
        $item = Get-Item -LiteralPath $file
        [ordered]@{
            relative_path = $name.Replace('\', '/')
            length_bytes = $item.Length
            sha256 = (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash
        }
    }
    return @($records)
}
function Read-Number([string]$Value) {
    $number = [double]::Parse($Value, [Globalization.NumberStyles]::Float,
        [Globalization.CultureInfo]::InvariantCulture)
    if ([double]::IsNaN($number) -or [double]::IsInfinity($number)) {
        throw 'Saved trajectory contains a non-finite numeric value.'
    }
    return $number
}

$hashesBefore = Get-InputHashes
$vehicle = Get-Content -LiteralPath (Join-Path $sourcePath $vehicleRelative) -Raw |
    ConvertFrom-Json
$wheelbase = [double]$vehicle.dimensions_m.wheelbase
if ($wheelbase -le 0 -or [double]::IsNaN($wheelbase) -or [double]::IsInfinity($wheelbase)) {
    throw 'Invalid wheelbase in the original Day 7 vehicle configuration.'
}
$limit = 4.9
$tolerance = 1e-9
$samples = @(Import-Csv -LiteralPath (Join-Path $sourcePath $csvRelative))
$groups = @($samples | Group-Object sample_vehicle_id)
$segments = [Collections.Generic.List[object]]::new()
$perVehicle = [Collections.Generic.List[object]]::new()
foreach ($group in $groups) {
    $points = @($group.Group | Sort-Object { [int]$_.sample_index })
    if ($points.Count -lt 2) { throw "Too few saved samples for $($group.Name)." }
    $vehicleSegments = [Collections.Generic.List[object]]::new()
    for ($i = 1; $i -lt $points.Count; $i++) {
        $a = $points[$i - 1]
        $b = $points[$i]
        $startSample = [int]$a.sample_index
        $endSample = [int]$b.sample_index
        if ($endSample -ne $startSample + 1) { throw 'Nonconsecutive saved sample indices.' }
        $t0 = Read-Number $a.time_s
        $t1 = Read-Number $b.time_s
        if ($t1 -le $t0) { throw 'Saved sample times are not strictly increasing.' }
        $v0 = Read-Number $a.speed_mps
        $v1 = Read-Number $b.speed_mps
        # The control recorded at sample k acts on edge k-1 -> k.
        $delta = Read-Number $b.steering_angle_rad
        $kappa = [Math]::Tan($delta) / $wheelbase
        $entryAy = $v0 * $v0 * [Math]::Abs($kappa)
        $exitAy = $v1 * $v1 * [Math]::Abs($kappa)
        $peakAy = [Math]::Max([Math]::Abs($v0), [Math]::Abs($v1))
        $peakAy = $peakAy * $peakAy * [Math]::Abs($kappa)
        $peakAtEntry = [Math]::Abs($v0) -ge [Math]::Abs($v1)
        $peakPoint = if ($peakAtEntry) { $a } else { $b }
        $entryFail = $entryAy -gt $limit + $tolerance
        $exitFail = $exitAy -gt $limit + $tolerance
        $violation = $peakAy -gt $limit + $tolerance
        $reason = if ($entryFail -and $exitFail) { 'entry_and_exit_lateral_acceleration_exceeded' }
            elseif ($entryFail) { 'entry_lateral_acceleration_exceeded' }
            elseif ($exitFail) { 'exit_lateral_acceleration_exceeded' }
            else { 'passed_lateral_peak_check' }
        $segment = [pscustomobject][ordered]@{
            vehicle_id = $group.Name
            priority = [int]$b.sample_priority
            segment_index = $i
            start_sample_index = $startSample
            end_sample_index = $endSample
            time_start_s = $t0
            time_end_s = $t1
            x_start_m = Read-Number $a.x_m
            y_start_m = Read-Number $a.y_m
            x_end_m = Read-Number $b.x_m
            y_end_m = Read-Number $b.y_m
            v_start_mps = $v0
            v_end_mps = $v1
            steering_angle_rad = $delta
            gear = [int]$b.gear
            curvature_per_m = $kappa
            entry_abs_ay_mps2 = $entryAy
            exit_abs_ay_mps2 = $exitAy
            peak_abs_ay_mps2 = $peakAy
            peak_at = if ($peakAtEntry) { 'entry' } else { 'exit' }
            peak_time_s = Read-Number $peakPoint.time_s
            peak_x_m = Read-Number $peakPoint.x_m
            peak_y_m = Read-Number $peakPoint.y_m
            peak_speed_mps = Read-Number $peakPoint.speed_mps
            exceeds_limit = $violation
            failure_reason = $reason
        }
        $segments.Add($segment)
        $vehicleSegments.Add($segment)
    }
    $failed = @($vehicleSegments | Where-Object exceeds_limit)
    $peak = $vehicleSegments | Sort-Object peak_abs_ay_mps2 -Descending | Select-Object -First 1
    $perVehicle.Add([pscustomobject][ordered]@{
        vehicle_id = $group.Name
        priority = [int]$points[0].sample_priority
        sample_count = $points.Count
        segment_count = $vehicleSegments.Count
        violating_segment_count = $failed.Count
        max_peak_abs_ay_mps2 = $peak.peak_abs_ay_mps2
        passes_lateral_peak_check = ($failed.Count -eq 0)
        violating_segment_indices = @($failed | ForEach-Object segment_index)
        maximum_peak_segment = $peak
    })
}
$hashesAfter = Get-InputHashes
$unchanged = (ConvertTo-Json -InputObject $hashesBefore -Depth 4 -Compress) -ceq
    (ConvertTo-Json -InputObject $hashesAfter -Depth 4 -Compress)
if (-not $unchanged) { throw 'Original baseline input hashes changed during diagnostic.' }
New-Item -ItemType Directory -Path $outputPath -Force | Out-Null
$allPath = Join-Path $outputPath 'segment_diagnostics.csv'
$failPath = Join-Path $outputPath 'violating_segments.csv'
$segments | Export-Csv -LiteralPath $allPath -NoTypeInformation -Encoding utf8
$violations = @($segments | Where-Object exceeds_limit)
if ($violations.Count -gt 0) {
    $violations | Export-Csv -LiteralPath $failPath -NoTypeInformation -Encoding utf8
} else {
    ($segments | Select-Object -First 1 | ConvertTo-Csv -NoTypeInformation)[0] |
        Set-Content -LiteralPath $failPath -Encoding utf8
}
$summary = [ordered]@{
    artifact_kind = 'legacy_saved_trajectory_diagnostic'
    diagnostic_method = 'independent_saved_csv_segment_peak_formula'
    generated_utc = [DateTime]::UtcNow.ToString('o')
    source_root = $sourcePath
    planning_order = @($perVehicle | Sort-Object priority | ForEach-Object vehicle_id)
    new_constrained_search_executed = $false
    matlab_executed = $false
    is_new_constrained_trajectory_acceptance = $false
    interpretation = 'Read-only diagnostic of original saved Day 7 samples; not a new constrained search result or trajectory acceptance.'
    control_convention = 'control(k) applies to edge k-1 -> k; new steering uses entry speed v(k-1)'
    formula = 'max(abs(v_start),abs(v_end))^2 * abs(tan(delta)/L)'
    max_lateral_acceleration_mps2 = $limit
    lateral_acceleration_tolerance_mps2 = $tolerance
    wheelbase_m = $wheelbase
    source_files_unchanged = $unchanged
    source_hashes_before = $hashesBefore
    source_hashes_after = $hashesAfter
    vehicle_count = $perVehicle.Count
    total_segment_count = $segments.Count
    total_violating_segment_count = $violations.Count
    per_vehicle = @($perVehicle.ToArray())
    all_segment_diagnostics_csv = $allPath
    violating_segments_csv = $failPath
}
$summary | ConvertTo-Json -Depth 12 |
    Set-Content -LiteralPath (Join-Path $outputPath 'summary.json') -Encoding utf8
$perVehicle | Select-Object vehicle_id, priority, sample_count, segment_count,
    violating_segment_count, max_peak_abs_ay_mps2 | Format-Table -AutoSize
Write-Output "SOURCE_FILES_UNCHANGED=$unchanged"
Write-Output "MATLAB_EXECUTED=False; NEW_CONSTRAINED_SEARCH_EXECUTED=False"
Write-Output "DIAGNOSTIC_SUMMARY=$(Join-Path $outputPath 'summary.json')"
