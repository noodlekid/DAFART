%% COMPREHENSIVE COVERAGE ANALYSIS WITH DATA EXTRACTION
% This script demonstrates:
%   1. Choosing appropriate resolution for your needs
%   2. Computing coverage efficiently
%   3. Extracting actionable information
%   4. Exporting data for mission planning
%
% Use this as your main workflow for rover mission planning

clear; close all; clc;

fprintf('=======================================================\n');
fprintf('  ROVER MISSION COVERAGE ANALYSIS\n');
fprintf('=======================================================\n\n');

%% ========================================================================
%  STEP 1: CHOOSE YOUR RESOLUTION
%  ========================================================================

% Get terrain area
terrain_area_km2 = (519341 - 518000) * (4251642 - 4250734) / 1e6;

% Choose analysis type based on your needs:
% 'quick'       - Fast preview for testing (1-2 min, 50m grid)
% 'standard'    - Good balance for most work (3-5 min, 20m grid)  <- RECOMMENDED
% 'detailed'    - High resolution for final plans (10-15 min, 10m grid)
% 'publication' - Very high res for reports (30+ min, 5m grid)

analysis_type = 'publication';  % CHANGE THIS BASED ON YOUR NEEDS

% Get recommended settings
res_config = CoverageAnalysisUtils.getRecommendedResolution(...
    terrain_area_km2, analysis_type);

fprintf('\nProceeding with %s resolution...\n', analysis_type);
input('Press Enter to continue...', 's');

%% ========================================================================
%  STEP 2: LOAD TERRAIN AND CONFIGURE SYSTEM
%  ========================================================================

% Terrain configuration
filename = 'mdrs_utah_geotif/mdrs.tif';
x_limits = [518000.00, 519341.00];
y_limits = [4250734.00, 4251642.00];

terrain = TerrainData(filename, x_limits, y_limits);

% Base station configuration
base_station = AntennaConfig(...
    'Position', [518500, 4251000, 1450], ...
    'Frequency', 2400e6, ...
    'Power',1, ...
    'Gain', 6, ...
    'PatternType', 'omnidirectional', ...
    'Name', 'Base_Station');

% Propagation model with recommended resolution
prop_model = RayTracingPropagation(terrain, ...
    'MaxRange', 3000, ...
    'RayResolution', res_config.RayResolution, ...  % Use recommended
    'GroundPermittivity', 4, ...
    'GroundConductivity', 0.005);

% Coverage mapper with recommended resolution
mapper = CoverageMapper(terrain, prop_model, ...
    'ReceiverHeight', 1.5, ...
    'ReceiverSensitivity', -65, ...
    'GridSpacing', res_config.GridSpacing);  % Use recommended

%% ========================================================================
%  STEP 3: COMPUTE BASE COVERAGE
%  ========================================================================

fprintf('\n--- Computing Base Station Coverage ---\n');
fprintf('This will take approximately %.0f minutes...\n', res_config.EstimatedTime_min);

tic;
base_coverage = mapper.computeCoverage(base_station);
elapsed = toc;

fprintf('Actual computation time: %.1f minutes\n', elapsed/60);

% Quick visualization
figure('Position', [100, 100, 1200, 800]);
mapper.visualizeCoverage(base_coverage);

%% ========================================================================
%  STEP 4: DEFINE YOUR MISSION
%  ========================================================================

fprintf('\n=== Mission Planning ===\n');

% CUSTOMIZE THESE FOR YOUR ACTUAL MISSION
mission_waypoints = [
    518300, 4251100;  % WP1: Starting point
    518400, 4251300;  % WP2: Science target A
    518600, 4251400;  % WP3: Science target B
    518800, 4251300;  % WP4: Ridge observation
    519000, 4251100;  % WP5: Distant target
    518900, 4250900;  % WP6: Return waypoint
    518700, 4250850;  % WP7: South exploration
    518500, 4251000;  % WP8: Return to base
];

% Labels for waypoints
waypoint_labels = {
    'Start', 'Science_A', 'Science_B', 'Ridge', ...
    'Distant', 'Return_1', 'South', 'Base'
};

fprintf('Mission defined with %d waypoints\n', size(mission_waypoints, 1));

%% ========================================================================
%  STEP 5: ANALYZE PATH COVERAGE (CRITICAL!)
%  ========================================================================

fprintf('\n=== Analyzing Mission Path Coverage ===\n');

% Analyze coverage along the entire path
path_analysis = CoverageAnalysisUtils.analyzePath(...
    base_coverage, mission_waypoints, 1);  % Sample every 10m

% Visualize path profile
fig_profile = CoverageAnalysisUtils.plotPathProfile(...
    path_analysis, base_coverage);

%% ========================================================================
%  STEP 6: QUERY SPECIFIC WAYPOINTS
%  ========================================================================

fprintf('\n=== Waypoint Coverage Details ===\n');

% Get detailed coverage at each waypoint
waypoint_results = CoverageAnalysisUtils.queryPoints(...
    base_coverage, mission_waypoints, waypoint_labels);

%% ========================================================================
%  STEP 7: IDENTIFY CRITICAL GAPS
%  ========================================================================

% Find which waypoints need relay support
critical_waypoints = [];
for i = 1:length(waypoint_results)
    if ~waypoint_results(i).IsCovered
        critical_waypoints = [critical_waypoints; i];
    elseif waypoint_results(i).Margin_dB < 5  % Less than 5dB margin
        fprintf('WARNING: %s has weak signal (%.1f dB margin)\n', ...
                waypoint_results(i).Label, waypoint_results(i).Margin_dB);
    end
end

if ~isempty(critical_waypoints)
    fprintf('\n*** RELAY REQUIRED ***\n');
    fprintf('Waypoints with no coverage:\n');
    for i = critical_waypoints'
        fprintf('  - %s at [%.0f, %.0f]\n', ...
                waypoint_labels{i}, mission_waypoints(i,1), mission_waypoints(i,2));
    end
else
    fprintf('\n✓ All waypoints have coverage!\n');
end

%% ========================================================================
%  STEP 8: PLAN RELAY IF NEEDED
%  ========================================================================

if ~isempty(critical_waypoints) || path_analysis.CoveragePercent < 90
    fprintf('\n=== Planning Relay Placement ===\n');
    
    % Create relay template
    relay_template = AntennaConfig(...
        'Position', [0, 0, 0], ...
        'Frequency', 915e6, ...
        'Power', 2, ...
        'Gain', 6, ...
        'PatternType', 'omnidirectional', ...
        'Name', 'Relay');
    
    % Create planner
    planner = RelayPlanner(mapper, relay_template);
    
    % Analyze gaps
    gap_analysis = planner.analyzeGaps(base_coverage, mission_waypoints);
    
    % Find relay candidates
    relay_candidates = planner.findRelayCandidates(...
        gap_analysis, base_station, 5);
    
    % Visualize
    fig_relay = planner.visualizeGapsAndCandidates(...
        gap_analysis, relay_candidates);
    figure(fig_relay);
    hold on;
    plot(mission_waypoints(:,1), mission_waypoints(:,2), ...
         'w*-', 'MarkerSize', 12, 'LineWidth', 2);
    
    % Evaluate best relay
    if ~isempty(relay_candidates)
        [best_relay, improved_coverage] = planner.evaluateRelayPlacement(...
            relay_candidates(1), base_coverage);
        
        % Re-analyze path with relay
        path_with_relay = CoverageAnalysisUtils.analyzePath(...
            improved_coverage, mission_waypoints, 10);
        
        fprintf('\n*** PATH COVERAGE COMPARISON ***\n');
        fprintf('Without relay: %.1f%% coverage\n', path_analysis.CoveragePercent);
        fprintf('With relay:    %.1f%% coverage\n', path_with_relay.CoveragePercent);
        fprintf('Improvement:   +%.1f%%\n', ...
                path_with_relay.CoveragePercent - path_analysis.CoveragePercent);
    end
else
    fprintf('\n✓ No relay needed - mission has good coverage!\n');
    relay_candidates = [];
end

%% ========================================================================
%  STEP 9: EXPORT DATA FOR MISSION USE
%  ========================================================================

fprintf('\n=== Exporting Mission Data ===\n');

% Export 1: Full coverage grid to CSV
CoverageAnalysisUtils.exportToCSV(base_coverage, 'coverage_grid.csv');

% Export 2: Relay recommendations (if any)
if exist('relay_candidates', 'var') && ~isempty(relay_candidates)
    CoverageAnalysisUtils.exportRelayRecommendations(...
        relay_candidates, 'relay_recommendations.csv');
end

% Export 3: Waypoint coverage report
fid = fopen('waypoint_coverage_report.txt', 'w');
fprintf(fid, 'WAYPOINT COVERAGE REPORT\n');
fprintf(fid, '========================\n\n');
fprintf(fid, 'Mission: Rover Traverse\n');
fprintf(fid, 'Date: %s\n\n', datestr(now));

for i = 1:length(waypoint_results)
    wp = waypoint_results(i);
    fprintf(fid, 'Waypoint: %s\n', wp.Label);
    fprintf(fid, '  Position: [%.1f E, %.1f N]\n', wp.Position(1), wp.Position(2));
    fprintf(fid, '  Rx Power: %.1f dBm\n', wp.RxPower_dBm);
    fprintf(fid, '  Margin: %.1f dB\n', wp.Margin_dB);
    fprintf(fid, '  LOS: %s\n', char(string(wp.IsLOS)));
    fprintf(fid, '  Status: %s\n', char(string(wp.IsCovered)));
    fprintf(fid, '\n');
end

fprintf(fid, '\nPATH STATISTICS\n');
fprintf(fid, '===============\n');
fprintf(fid, 'Total distance: %.2f km\n', path_analysis.TotalDistance_m/1000);
fprintf(fid, 'Coverage: %.1f%%\n', path_analysis.CoveragePercent);
fprintf(fid, 'Number of gaps: %d\n', length(path_analysis.Gaps));

if ~isempty(path_analysis.Gaps)
    fprintf(fid, '\nCOVERAGE GAPS:\n');
    for g = 1:length(path_analysis.Gaps)
        gap = path_analysis.Gaps(g);
        fprintf(fid, '  Gap %d: %.0f m from [%.1f, %.1f] to [%.1f, %.1f]\n', ...
                g, gap.Length_m, ...
                gap.StartPos(1), gap.StartPos(2), ...
                gap.EndPos(1), gap.EndPos(2));
    end
end

fclose(fid);
fprintf('Waypoint report saved to: waypoint_coverage_report.txt\n');

% Export 4: Save MATLAB workspace
save('mission_coverage_analysis.mat', 'base_coverage', 'mission_waypoints', ...
     'waypoint_results', 'path_analysis', 'res_config');
fprintf('MATLAB workspace saved to: mission_coverage_analysis.mat\n');

%% ========================================================================
%  STEP 10: GENERATE EXECUTIVE SUMMARY
%  ========================================================================

fprintf('\n');
fprintf('========================================================\n');
fprintf('           MISSION COVERAGE EXECUTIVE SUMMARY           \n');
fprintf('========================================================\n\n');

fprintf('MISSION PARAMETERS:\n');
fprintf('  Waypoints: %d\n', size(mission_waypoints, 1));
fprintf('  Total path: %.2f km\n', path_analysis.TotalDistance_m/1000);
fprintf('  Analysis resolution: %s (%.0f m grid)\n', ...
        analysis_type, res_config.GridSpacing);

fprintf('\nCOVERAGE RESULTS:\n');
fprintf('  Area coverage: %.1f%% (%.3f / %.3f km²)\n', ...
        base_coverage.Statistics.CoveragePercent, ...
        base_coverage.Statistics.CoveredArea_km2, ...
        base_coverage.Statistics.TotalArea_km2);
fprintf('  Path coverage: %.1f%% (%.2f / %.2f km)\n', ...
        path_analysis.CoveragePercent, ...
        path_analysis.TotalDistance_m * path_analysis.CoveragePercent / 100000, ...
        path_analysis.TotalDistance_m / 1000);

covered_wp = sum([waypoint_results.IsCovered]);
fprintf('  Waypoints covered: %d / %d (%.0f%%)\n', ...
        covered_wp, length(waypoint_results), ...
        100 * covered_wp / length(waypoint_results));

fprintf('\nRECOMMENDATIONS:\n');
if covered_wp == length(waypoint_results) && path_analysis.CoveragePercent > 95
    fprintf('  ✓ MISSION READY - Excellent coverage throughout\n');
    fprintf('  ✓ No relay required\n');
elseif covered_wp == length(waypoint_results) && path_analysis.CoveragePercent > 85
    fprintf('  ⚠ MISSION VIABLE - Some gaps between waypoints\n');
    fprintf('  • Consider relay for 100%% path coverage\n');
else
    fprintf('  ✗ RELAY REQUIRED - Critical coverage gaps\n');
    if exist('relay_candidates', 'var') && ~isempty(relay_candidates)
        fprintf('  • Deploy relay at: [%.0f E, %.0f N]\n', ...
                relay_candidates(1).Position(1), relay_candidates(1).Position(2));
        fprintf('  • Mast height: %.0f m\n', relay_candidates(1).RelayHeight);
    end
end

fprintf('\nOUTPUT FILES:\n');
fprintf('  • coverage_grid.csv - Full coverage data\n');
fprintf('  • waypoint_coverage_report.txt - Waypoint details\n');
fprintf('  • mission_coverage_analysis.mat - MATLAB data\n');
if exist('relay_candidates', 'var') && ~isempty(relay_candidates)
    fprintf('  • relay_recommendations.csv - Relay locations\n');
end

fprintf('\n========================================================\n\n');

%% ========================================================================
%  RESOLUTION COMPARISON (OPTIONAL)
%  ========================================================================

% Uncomment to compare different resolutions
% fprintf('\n=== Resolution Comparison ===\n');
% fprintf('Running quick analysis for comparison...\n');
% 
% mapper_quick = CoverageMapper(terrain, prop_model, ...
%     'ReceiverSensitivity', -100, 'GridSpacing', 50);
% 
% tic;
% coverage_quick = mapper_quick.computeCoverage(base_station);
% time_quick = toc;
% 
% fprintf('\nComparison:\n');
% fprintf('Quick (50m):    %.1f%% coverage in %.1f min\n', ...
%         coverage_quick.Statistics.CoveragePercent, time_quick/60);
% fprintf('Standard (20m): %.1f%% coverage in %.1f min\n', ...
%         base_coverage.Statistics.CoveragePercent, elapsed/60);