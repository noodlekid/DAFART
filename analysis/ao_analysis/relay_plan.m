%% RELAY PLACEMENT PLANNING - EXAMPLE SCRIPT
% This script demonstrates how to use the RelayPlanner to identify
% optimal relay/repeater locations for extending rover coverage
%
% WORKFLOW:
%   1. Compute base station coverage
%   2. Define mission area (rover waypoints)
%   3. Analyze coverage gaps
%   4. Find relay candidate locations
%   5. Evaluate and select best relay
%   6. Visualize improved coverage

clear; close all; clc;

fprintf('=======================================================\n');
fprintf('  RELAY PLACEMENT PLANNING FOR LUNAR ROVER\n');
fprintf('=======================================================\n\n');

%% ========================================================================
%  STEP 1: LOAD TERRAIN AND SETUP BASE STATION
%  ========================================================================

% Terrain configuration
filename = 'mdrs_utah_geotif/mdrs.tif';
min_easting_m = 518000.00;
max_easting_m = 519341.00;
min_northing_m = 4250734.00;
max_northing_m = 4251642.00;

x_limits = [min_easting_m, max_easting_m];
y_limits = [min_northing_m, max_northing_m];

% Load terrain
terrain = TerrainData(filename, x_limits, y_limits);

% Configure base station
% Now you only need x, y location + height above ground!
base_station = AntennaConfig(...
    'Location', [518500, 4251000], ...   % Easting, Northing
    'HeightAGL', 10, ...                  % 10m above ground
    'Frequency', 915e6, ...
    'Power', 5, ...
    'Gain', 6, ...
    'PatternType', 'omnidirectional', ...
    'Name', 'Base_Station');

% Setup propagation model
prop_model = RayTracingPropagation(terrain, ...
    'MaxRange', 3000, ...
    'RayResolution', 10, ...
    'GroundPermittivity', 4, ...
    'GroundConductivity', 0.005);

% Create coverage mapper
mapper = CoverageMapper(terrain, prop_model, ...
    'ReceiverHeight', 2.0, ...      % Rover antenna height
    'ReceiverSensitivity', -100, ... % Minimum signal for rover
    'GridSpacing', 20);

%% ========================================================================
%  STEP 2: COMPUTE BASE STATION COVERAGE
%  ========================================================================

fprintf('\n--- Computing Base Station Coverage ---\n');
base_coverage = mapper.computeCoverage(base_station);

% Visualize base coverage
figure('Position', [100, 100, 1200, 800]);
mapper.visualizeCoverage(base_coverage);
title('Base Station Coverage - Before Relay');

%% ========================================================================
%  STEP 3: DEFINE MISSION AREA (ROVER WAYPOINTS)
%  ========================================================================

fprintf('\n--- Defining Mission Area ---\n');

% Define your rover's planned mission waypoints
% Replace these with your actual mission coordinates
mission_waypoints = [
    518300, 4251100;  % Waypoint 1
    518200, 4251200;  % Waypoint 2
    518400, 4251300;  % Waypoint 3
    518600, 4251400;  % Waypoint 4
    518800, 4251200;  % Waypoint 5
    519000, 4251100;  % Waypoint 6
    519100, 4250900;  % Waypoint 7
    518900, 4250800;  % Waypoint 8
];

fprintf('Mission has %d waypoints\n', size(mission_waypoints, 1));

% Check which waypoints are currently covered
fprintf('\nChecking waypoint coverage:\n');
for i = 1:size(mission_waypoints, 1)
    x = mission_waypoints(i, 1);
    y = mission_waypoints(i, 2);
    z_terrain = terrain.getElevation(x, y);
    rx_pos = [x, y, z_terrain + 2.0];
    
    [path_loss, is_los] = prop_model.computePathLoss(...
        base_station.Position, rx_pos, base_station.Frequency);
    
    P_tx_dBm = 10*log10(base_station.Power * 1000);
    P_rx = P_tx_dBm + base_station.Gain - path_loss;
    
    is_covered = P_rx >= mapper.ReceiverSensitivity;
    
    fprintf('  WP%d [%.0f, %.0f]: Rx=%.1f dBm, LOS=%d, Covered=%d\n', ...
            i, x, y, P_rx, is_los, is_covered);
end

%% ========================================================================
%  STEP 4: ANALYZE COVERAGE GAPS
%  ========================================================================

% Create template relay configuration
relay_template = AntennaConfig(...
    'Location', [0, 0], ...      % Will be set by planner
    'HeightAGL', 5, ...          % 5m mast for relay
    'Frequency', 915e6, ...
    'Power', 2, ...              % Lower power for relay
    'Gain', 6, ...
    'PatternType', 'omnidirectional', ...
    'Name', 'Relay');

% Create relay planner
planner = RelayPlanner(mapper, relay_template, ...
    'MinRelayHeight', 3, ...     % Minimum 3m mast
    'MaxRelayHeight', 10);       % Maximum 10m mast

% Analyze gaps with mission area priority
gap_analysis = planner.analyzeGaps(base_coverage, mission_waypoints);

%% ========================================================================
%  STEP 5: FIND RELAY CANDIDATE LOCATIONS
%  ========================================================================

% Find top candidate locations for relays
relay_candidates = planner.findRelayCandidates(...
    gap_analysis, base_station, 5);  % 5 candidates per gap

%% ========================================================================
%  STEP 6: VISUALIZE GAPS AND CANDIDATES
%  ========================================================================

fprintf('\n--- Generating Visualization ---\n');
fig_gaps = planner.visualizeGapsAndCandidates(...
    gap_analysis, relay_candidates);

% Overlay mission waypoints
figure(fig_gaps);
hold on;
plot(mission_waypoints(:,1), mission_waypoints(:,2), ...
     'w*-', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Mission Path');
for i = 1:size(mission_waypoints, 1)
    text(mission_waypoints(i,1) + 30, mission_waypoints(i,2), ...
         sprintf('WP%d', i), 'Color', 'white', 'FontWeight', 'bold');
end
legend('show');

%% ========================================================================
%  STEP 7: EVALUATE TOP RELAY CANDIDATE
%  ========================================================================

if ~isempty(relay_candidates)
    fprintf('\n--- Evaluating Best Relay Candidate ---\n');
    
    % Evaluate the top candidate
    best_candidate = relay_candidates(1);
    
    [best_relay, improved_coverage] = planner.evaluateRelayPlacement(...
        best_candidate, base_coverage);
    
    % Visualize improved coverage
    figure('Position', [100, 100, 1200, 800]);
    mapper.visualizeCoverage(improved_coverage);
    title(sprintf('Coverage After Adding Relay #%d', best_candidate.ID));
    hold on;
    
    % Show relay location
    plot(best_relay.Position(1), best_relay.Position(2), ...
         'co', 'MarkerSize', 20, 'MarkerFaceColor', 'cyan', ...
         'LineWidth', 3, 'DisplayName', 'Relay');
    
    % Show mission waypoints
    plot(mission_waypoints(:,1), mission_waypoints(:,2), ...
         'w*-', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Mission Path');
    
    legend('show');
    
    % Check improved waypoint coverage
    fprintf('\nRe-checking waypoint coverage after relay:\n');
    covered_count = 0;
    for i = 1:size(mission_waypoints, 1)
        x = mission_waypoints(i, 1);
        y = mission_waypoints(i, 2);
        
        % Interpolate from improved coverage map
        is_covered = interp2(improved_coverage.X, improved_coverage.Y, ...
                            double(improved_coverage.Coverage), ...
                            x, y, 'nearest');
        
        P_rx = interp2(improved_coverage.X, improved_coverage.Y, ...
                      improved_coverage.ReceivedPower_dBm, ...
                      x, y, 'linear');
        
        if is_covered
            covered_count = covered_count + 1;
        end
        
        fprintf('  WP%d: Rx=%.1f dBm, Covered=%d\n', i, P_rx, is_covered);
    end
    
    fprintf('\nMission waypoints covered: %d / %d (%.1f%%)\n', ...
            covered_count, size(mission_waypoints, 1), ...
            100*covered_count/size(mission_waypoints, 1));
end

%% ========================================================================
%  STEP 8: GENERATE COMPREHENSIVE REPORT
%  ========================================================================

fprintf('\n--- Generating Relay Placement Report ---\n');
report = planner.generateRelayReport(...
    gap_analysis, relay_candidates, mission_waypoints);

%% ========================================================================
%  STEP 9: EVALUATE MULTIPLE RELAYS (OPTIONAL)
%  ========================================================================

fprintf('\n--- Evaluating Multiple Relay Strategy ---\n');

% If you need more coverage, evaluate adding multiple relays
if length(relay_candidates) > 1
    % Start with base coverage
    multi_coverage = base_coverage;
    relay_configs = {};
    
    % Add relays iteratively (up to 3)
    num_relays_to_add = min(3, length(relay_candidates));
    
    for r = 1:num_relays_to_add
        candidate = relay_candidates(r);
        
        % Create relay
        relay = relay_template;
        relay.Position = candidate.Position;
        relay.Name = sprintf('Relay_%d', r);
        
        % Compute relay coverage
        relay_cov = mapper.computeCoverage(relay);
        
        % Combine with existing
        multi_coverage = mapper.combineCoverage({multi_coverage, relay_cov});
        relay_configs{r} = relay;
        
        fprintf('After adding Relay %d: %.1f%% coverage\n', ...
                r, multi_coverage.Statistics.CoveragePercent);
    end
    
    % Visualize multi-relay coverage
    figure('Position', [100, 100, 1200, 800]);
    mapper.visualizeCoverage(multi_coverage);
    title(sprintf('Coverage with %d Relays', num_relays_to_add));
    hold on;
    
    % Show all relay locations
    for r = 1:length(relay_configs)
        plot(relay_configs{r}.Position(1), relay_configs{r}.Position(2), ...
             'co', 'MarkerSize', 16, 'MarkerFaceColor', 'cyan', 'LineWidth', 2);
        text(relay_configs{r}.Position(1) + 30, relay_configs{r}.Position(2), ...
             sprintf('R%d', r), 'Color', 'cyan', 'FontWeight', 'bold');
    end
    
    % Show mission waypoints
    plot(mission_waypoints(:,1), mission_waypoints(:,2), ...
         'w*-', 'MarkerSize', 12, 'LineWidth', 2);
end

%% ========================================================================
%  STEP 10: EXPORT RESULTS
%  ========================================================================

fprintf('\n--- Saving Results ---\n');

% Save relay recommendations
save('relay_placement_results.mat', 'relay_candidates', 'gap_analysis', ...
     'best_candidate', 'improved_coverage', 'mission_waypoints', 'report');

fprintf('Results saved to: relay_placement_results.mat\n');

% Export improved coverage map
% mapper.exportToGeoTIFF(improved_coverage, 'coverage_with_relay.tif');

fprintf('\n=======================================================\n');
fprintf('  RELAY PLANNING COMPLETE\n');
fprintf('=======================================================\n\n');

%% ========================================================================
%  PRACTICAL DEPLOYMENT GUIDE
%  ========================================================================

fprintf('\n========== DEPLOYMENT RECOMMENDATIONS ==========\n\n');

if ~isempty(relay_candidates)
    fprintf('PRIORITY 1 RELAY LOCATION:\n');
    fprintf('  Coordinates: [%.1f E, %.1f N] (UTM)\n', ...
            best_candidate.Position(1), best_candidate.Position(2));
    fprintf('  Elevation MSL: %.1f m\n', best_candidate.Position(3));
    fprintf('  Antenna height: %.1f m above ground\n', best_candidate.RelayHeight);
    fprintf('  Expected Rx from base: %.1f dBm\n', best_candidate.RxPowerFromBase_dBm);
    fprintf('  Coverage improvement: +%.1f%%\n', ...
            improved_coverage.Statistics.CoveragePercent - ...
            base_coverage.Statistics.CoveragePercent);
    
    fprintf('\nDEPLOYMENT CHECKLIST:\n');
    fprintf('  [ ] Verify site accessibility\n');
    fprintf('  [ ] Confirm actual terrain at site\n');
    fprintf('  [ ] Install %s mast/antenna mount\n', ...
            sprintf('%.1fm', best_candidate.RelayHeight));
    fprintf('  [ ] Orient antenna for optimal pattern\n');
    fprintf('  [ ] Test link quality to base station\n');
    fprintf('  [ ] Verify rover connectivity at key waypoints\n');
end

fprintf('\n================================================\n\n');