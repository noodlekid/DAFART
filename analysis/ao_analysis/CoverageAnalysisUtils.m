classdef CoverageAnalysisUtils
    % COVERAGEANALYSISUTILS Utility functions for extracting meaningful coverage data
    % Provides tools for path analysis, point queries, and data export
    
    methods (Static)
        
        function config = getRecommendedResolution(terrain_area_km2, analysis_type)
            % Get recommended resolution settings based on area and analysis type
            % INPUTS:
            %   terrain_area_km2: area of terrain in km²
            %   analysis_type: 'quick', 'standard', 'detailed', or 'publication'
            % OUTPUT:
            %   config: struct with recommended settings
            
            fprintf('\n=== Resolution Recommendations ===\n');
            fprintf('Terrain area: %.2f km²\n', terrain_area_km2);
            fprintf('Analysis type: %s\n', analysis_type);
            
            config = struct();
            
            switch lower(analysis_type)
                case 'quick'
                    % Fast preview - good for initial testing
                    config.GridSpacing = 50;           % meters
                    config.RayResolution = 20;         % meters
                    config.Description = 'Quick preview (1-2 min)';
                    config.EstimatedTime_min = 1;
                    
                case 'standard'
                    % Balanced - good for most analyses
                    config.GridSpacing = 20;           % meters
                    config.RayResolution = 10;         % meters
                    config.Description = 'Standard analysis (3-5 min)';
                    config.EstimatedTime_min = 4;
                    
                case 'detailed'
                    % High resolution - for final planning
                    config.GridSpacing = 10;           % meters
                    config.RayResolution = 5;          % meters
                    config.Description = 'Detailed analysis (10-15 min)';
                    config.EstimatedTime_min = 12;
                    
                case 'publication'
                    % Very high resolution - for reports/papers
                    config.GridSpacing = 5;            % meters
                    config.RayResolution = 3;          % meters
                    config.Description = 'Publication quality (30+ min)';
                    config.EstimatedTime_min = 35;
                    
                otherwise
                    error('Unknown analysis type. Use: quick, standard, detailed, or publication');
            end
            
            % Adjust for large areas
            if terrain_area_km2 > 2.0
                config.GridSpacing = config.GridSpacing * 1.5;
                config.EstimatedTime_min = config.EstimatedTime_min * 1.5;
                fprintf('Note: Large area detected, adjusted spacing to %.0f m\n', ...
                        config.GridSpacing);
            end
            
            % Calculate grid size
            area_m2 = terrain_area_km2 * 1e6;
            grid_points = area_m2 / config.GridSpacing^2;
            
            fprintf('\nRecommended settings:\n');
            fprintf('  Grid spacing: %.0f m\n', config.GridSpacing);
            fprintf('  Ray resolution: %.0f m\n', config.RayResolution);
            fprintf('  Grid points: ~%.0f\n', grid_points);
            fprintf('  Estimated time: ~%.0f minutes\n', config.EstimatedTime_min);
            fprintf('  %s\n', config.Description);
        end
        
        function coverage_data = analyzePath(coverage_map, path_waypoints, spacing)
            % Analyze coverage along a specific path
            % INPUTS:
            %   coverage_map: output from CoverageMapper.computeCoverage
            %   path_waypoints: Nx2 array of [x,y] coordinates defining path
            %   spacing: sample spacing along path in meters (default: 10m)
            % OUTPUT:
            %   coverage_data: struct with path analysis
            
            if nargin < 3
                spacing = 10;
            end
            
            fprintf('\n=== Path Coverage Analysis ===\n');
            
            % Generate points along path
            total_distance = 0;
            path_points = [];
            segment_distances = [];
            
            for i = 1:size(path_waypoints, 1)-1
                p1 = path_waypoints(i, :);
                p2 = path_waypoints(i+1, :);
                
                segment_length = norm(p2 - p1);
                segment_distances(i) = segment_length;
                total_distance = total_distance + segment_length;
                
                num_samples = ceil(segment_length / spacing);
                x_seg = linspace(p1(1), p2(1), num_samples);
                y_seg = linspace(p1(2), p2(2), num_samples);
                
                path_points = [path_points; [x_seg(:), y_seg(:)]];
            end
            
            fprintf('Path length: %.2f km\n', total_distance/1000);
            fprintf('Sample points: %d (every %.0f m)\n', size(path_points,1), spacing);
            
            % Sample coverage at path points
            rx_power = interp2(coverage_map.X, coverage_map.Y, ...
                              coverage_map.ReceivedPower_dBm, ...
                              path_points(:,1), path_points(:,2), 'linear');
            
            is_covered = rx_power >= coverage_map.ReceiverSensitivity_dBm;
            is_los = interp2(coverage_map.X, coverage_map.Y, ...
                            double(coverage_map.LineOfSight), ...
                            path_points(:,1), path_points(:,2), 'nearest');
            
            % Calculate statistics
            coverage_percent = 100 * sum(is_covered) / length(is_covered);
            los_percent = 100 * sum(is_los) / length(is_los);
            
            % Find coverage gaps along path
            gap_starts = find(diff([1; is_covered; 1]) == -1);
            gap_ends = find(diff([1; is_covered; 1]) == 1) - 1;
            
            gaps = [];
            for g = 1:length(gap_starts)
                gap_length = (gap_ends(g) - gap_starts(g) + 1) * spacing;
                gap_start_pos = path_points(gap_starts(g), :);
                gap_end_pos = path_points(gap_ends(g), :);
                
                gaps(g).StartPos = gap_start_pos;
                gaps(g).EndPos = gap_end_pos;
                gaps(g).Length_m = gap_length;
                gaps(g).StartIndex = gap_starts(g);
                gaps(g).EndIndex = gap_ends(g);
            end
            
            fprintf('\n--- Path Statistics ---\n');
            fprintf('Coverage: %.1f%% of path length\n', coverage_percent);
            fprintf('Line-of-sight: %.1f%% of path\n', los_percent);
            fprintf('Number of gaps: %d\n', length(gaps));
            fprintf('Min Rx power: %.1f dBm\n', min(rx_power));
            fprintf('Max Rx power: %.1f dBm\n', max(rx_power));
            fprintf('Avg Rx power: %.1f dBm\n', mean(rx_power));
            
            if ~isempty(gaps)
                fprintf('\nCoverage Gaps:\n');
                for g = 1:min(5, length(gaps))
                    fprintf('  Gap %d: %.0f m at [%.1f, %.1f] to [%.1f, %.1f]\n', ...
                            g, gaps(g).Length_m, ...
                            gaps(g).StartPos(1), gaps(g).StartPos(2), ...
                            gaps(g).EndPos(1), gaps(g).EndPos(2));
                end
            end
            
            % Package results
            coverage_data = struct();
            coverage_data.PathPoints = path_points;
            coverage_data.RxPower_dBm = rx_power;
            coverage_data.IsCovered = is_covered;
            coverage_data.IsLOS = is_los;
            coverage_data.TotalDistance_m = total_distance;
            coverage_data.CoveragePercent = coverage_percent;
            coverage_data.LOSPercent = los_percent;
            coverage_data.Gaps = gaps;
            coverage_data.Waypoints = path_waypoints;
        end
        
        function query_results = queryPoints(coverage_map, query_points, labels)
            % Query coverage at specific points of interest
            % INPUTS:
            %   coverage_map: output from CoverageMapper.computeCoverage
            %   query_points: Nx2 array of [x,y] coordinates
            %   labels: cell array of labels for each point (optional)
            % OUTPUT:
            %   query_results: struct array with results for each point
            
            if nargin < 3
                labels = arrayfun(@(i) sprintf('Point_%d', i), ...
                                 1:size(query_points,1), 'UniformOutput', false);
            end
            
            fprintf('\n=== Point Coverage Query ===\n');
            fprintf('Querying %d points...\n\n', size(query_points, 1));
            
            query_results = struct('Label', {}, 'Position', {}, ...
                                  'RxPower_dBm', {}, 'IsCovered', {}, ...
                                  'IsLOS', {}, 'Margin_dB', {});
            
            for i = 1:size(query_points, 1)
                x = query_points(i, 1);
                y = query_points(i, 2);
                
                % Interpolate coverage data
                rx_power = interp2(coverage_map.X, coverage_map.Y, ...
                                  coverage_map.ReceivedPower_dBm, ...
                                  x, y, 'linear');
                
                is_covered = rx_power >= coverage_map.ReceiverSensitivity_dBm;
                
                is_los = interp2(coverage_map.X, coverage_map.Y, ...
                                double(coverage_map.LineOfSight), ...
                                x, y, 'nearest');
                
                margin = rx_power - coverage_map.ReceiverSensitivity_dBm;
                
                query_results(i).Label = labels{i};
                query_results(i).Position = [x, y];
                query_results(i).RxPower_dBm = rx_power;
                query_results(i).IsCovered = is_covered;
                query_results(i).IsLOS = logical(is_los);
                query_results(i).Margin_dB = margin;
                
                % Print result
                status = '';
                if is_covered
                    if margin > 10
                        status = 'EXCELLENT';
                    elseif margin > 5
                        status = 'GOOD';
                    else
                        status = 'MARGINAL';
                    end
                else
                    status = 'NO COVERAGE';
                end
                
                fprintf('%s: [%.1f, %.1f]\n', labels{i}, x, y);
                fprintf('  Rx Power: %.1f dBm (%s)\n', rx_power, status);
                fprintf('  Margin: %.1f dB\n', margin);
                fprintf('  LOS: %s\n\n', char(string(logical(is_los))));
            end
        end
        
        function exportToCSV(coverage_map, filename)
            % Export coverage data to CSV for external analysis
            % INPUTS:
            %   coverage_map: output from CoverageMapper.computeCoverage
            %   filename: output CSV filename
            
            fprintf('\n=== Exporting to CSV ===\n');
            fprintf('File: %s\n', filename);
            
            % Flatten grids
            x_flat = coverage_map.X(:);
            y_flat = coverage_map.Y(:);
            z_terrain = coverage_map.Z_terrain(:);
            z_rx = coverage_map.Z_receiver(:);
            rx_power = coverage_map.ReceivedPower_dBm(:);
            path_loss = coverage_map.PathLoss_dB(:);
            is_covered = double(coverage_map.Coverage(:));
            is_los = double(coverage_map.LineOfSight(:));
            
            % Create table
            T = table(x_flat, y_flat, z_terrain, z_rx, rx_power, path_loss, ...
                     is_covered, is_los, ...
                     'VariableNames', {'Easting_m', 'Northing_m', ...
                                      'TerrainElev_m', 'RxHeight_m', ...
                                      'RxPower_dBm', 'PathLoss_dB', ...
                                      'IsCovered', 'IsLOS'});
            
            % Remove NaN/Inf rows
            valid_rows = isfinite(rx_power);
            T = T(valid_rows, :);
            
            % Write to CSV
            writetable(T, filename);
            
            fprintf('Exported %d data points\n', height(T));
            fprintf('Done!\n');
        end
        
        function exportRelayRecommendations(relay_candidates, filename)
            % Export relay recommendations to CSV
            % INPUTS:
            %   relay_candidates: output from RelayPlanner.findRelayCandidates
            %   filename: output CSV filename
            
            fprintf('\n=== Exporting Relay Recommendations ===\n');
            fprintf('File: %s\n', filename);
            
            if isempty(relay_candidates)
                fprintf('No relay candidates to export\n');
                return;
            end
            
            % Extract data
            ids = [relay_candidates.ID]';
            gap_ids = [relay_candidates.GapID]';
            x = arrayfun(@(c) c.Position(1), relay_candidates)';
            y = arrayfun(@(c) c.Position(2), relay_candidates)';
            z = arrayfun(@(c) c.Position(3), relay_candidates)';
            z_terrain = [relay_candidates.TerrainElevation]';
            relay_height = [relay_candidates.RelayHeight]';
            score = [relay_candidates.Score]';
            dist_to_gap = [relay_candidates.DistanceToGap_m]';
            rx_power = [relay_candidates.RxPowerFromBase_dBm]';
            los = double([relay_candidates.LOStoBase]');
            
            % Create table
            T = table(ids, gap_ids, x, y, z, z_terrain, relay_height, score, ...
                     dist_to_gap, rx_power, los, ...
                     'VariableNames', {'CandidateID', 'GapID', ...
                                      'Easting_m', 'Northing_m', 'Elevation_MSL_m', ...
                                      'TerrainElev_m', 'RelayHeight_m', ...
                                      'Score', 'DistToGap_m', 'RxFromBase_dBm', 'LOS'});
            
            % Write to CSV
            writetable(T, filename);
            
            fprintf('Exported %d relay candidates\n', length(relay_candidates));
            fprintf('Done!\n');
        end
        
        function fig = plotPathProfile(path_coverage_data, coverage_map)
            % Plot elevation and coverage profile along path
            % INPUTS:
            %   path_coverage_data: output from analyzePath
            %   coverage_map: original coverage map
            
            fig = figure('Position', [100, 100, 1400, 600]);
            
            % Calculate cumulative distance
            path_points = path_coverage_data.PathPoints;
            distances = [0; cumsum(sqrt(sum(diff(path_points).^2, 2)))];
            distances_km = distances / 1000;
            
            % Get terrain elevation along path
            z_terrain = interp2(coverage_map.X, coverage_map.Y, ...
                               coverage_map.Z_terrain, ...
                               path_points(:,1), path_points(:,2), 'linear');
            
            % Subplot 1: Elevation profile
            subplot(2,1,1);
            plot(distances_km, z_terrain, 'k-', 'LineWidth', 2);
            xlabel('Distance along path (km)');
            ylabel('Elevation (m)');
            title('Terrain Elevation Profile');
            grid on;
            
            % Subplot 2: Received power and coverage
            subplot(2,1,2);
            yyaxis left;
            plot(distances_km, path_coverage_data.RxPower_dBm, 'b-', 'LineWidth', 2);
            hold on;
            yline(coverage_map.ReceiverSensitivity_dBm, 'r--', 'LineWidth', 2, ...
                  'Label', 'Sensitivity');
            ylabel('Received Power (dBm)');
            
            yyaxis right;
            plot(distances_km, double(path_coverage_data.IsCovered), 'g-', 'LineWidth', 2);
            ylabel('Coverage (1=Yes, 0=No)');
            ylim([-0.1, 1.1]);
            
            xlabel('Distance along path (km)');
            title('Coverage Profile Along Path');
            grid on;
            legend('Rx Power', 'Sensitivity', 'Covered', 'Location', 'best');
            
            % Highlight gaps
            if ~isempty(path_coverage_data.Gaps)
                subplot(2,1,2);
                yyaxis left;
                for g = 1:length(path_coverage_data.Gaps)
                    gap = path_coverage_data.Gaps(g);
                    gap_dist = distances([gap.StartIndex, gap.EndIndex]) / 1000;
                    patch([gap_dist(1), gap_dist(2), gap_dist(2), gap_dist(1)], ...
                          [min(ylim), min(ylim), max(ylim), max(ylim)], ...
                          'red', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
                end
            end
        end
        
        function exportKML(coverage_map, filename, utm_zone)
            % Export coverage to KML for Google Earth (basic implementation)
            % INPUTS:
            %   coverage_map: output from CoverageMapper.computeCoverage
            %   filename: output KML filename
            %   utm_zone: UTM zone number (e.g., 12 for Utah)
            
            fprintf('\n=== Exporting to KML ===\n');
            fprintf('File: %s\n', filename);
            fprintf('UTM Zone: %d\n', utm_zone);
            
            % This is a simplified KML export
            % For production use, consider using proper UTM to Lat/Lon conversion
            fprintf('Note: KML export requires UTM to Lat/Lon conversion\n');
            fprintf('Consider using external tools or MATLAB Mapping Toolbox functions\n');
            fprintf('Example: [lat, lon] = utm2deg(easting, northing, zone)\n');
        end
        
        function comparison = compareConfigurations(coverage_maps, config_names)
            % Compare multiple antenna configurations side-by-side
            % INPUTS:
            %   coverage_maps: cell array of coverage map structs
            %   config_names: cell array of configuration names
            % OUTPUT:
            %   comparison: struct with comparison data
            
            fprintf('\n=== Configuration Comparison ===\n\n');
            
            num_configs = length(coverage_maps);
            
            fprintf('%-20s | %10s | %10s | %10s\n', ...
                    'Configuration', 'Coverage %', 'LOS %', 'Area km²');
            fprintf('%s\n', repmat('-', 1, 60));
            
            comparison = struct();
            comparison.Names = config_names;
            comparison.CoveragePercent = zeros(num_configs, 1);
            comparison.LOSPercent = zeros(num_configs, 1);
            comparison.CoveredArea_km2 = zeros(num_configs, 1);
            
            for i = 1:num_configs
                cmap = coverage_maps{i};
                
                comparison.CoveragePercent(i) = cmap.Statistics.CoveragePercent;
                comparison.LOSPercent(i) = cmap.Statistics.LOSPercent;
                comparison.CoveredArea_km2(i) = cmap.Statistics.CoveredArea_km2;
                
                fprintf('%-20s | %9.1f%% | %9.1f%% | %9.3f\n', ...
                        config_names{i}, ...
                        comparison.CoveragePercent(i), ...
                        comparison.LOSPercent(i), ...
                        comparison.CoveredArea_km2(i));
            end
            
            fprintf('\n');
        end
    end
end