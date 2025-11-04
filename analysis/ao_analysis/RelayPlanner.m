classdef RelayPlanner
    % RELAYPLANNER Automated relay/repeater placement optimization
    % Identifies coverage gaps and suggests optimal relay locations
    
    properties
        CoverageMapper      % CoverageMapper object
        TerrainData        % TerrainData object
        PropagationModel   % Propagation model
        MinRelayHeight     % Minimum relay height above ground (m)
        MaxRelayHeight     % Maximum relay height above ground (m)
        RelayConfig        % Template antenna config for relays
    end
    
    methods
        function obj = RelayPlanner(coverage_mapper, relay_config, varargin)
            % Constructor
            p = inputParser;
            addRequired(p, 'coverage_mapper', @(x) isa(x, 'CoverageMapper'));
            addRequired(p, 'relay_config', @(x) isa(x, 'AntennaConfig'));
            addParameter(p, 'MinRelayHeight', 3, @isnumeric);
            addParameter(p, 'MaxRelayHeight', 10, @isnumeric);
            
            parse(p, coverage_mapper, relay_config, varargin{:});
            
            obj.CoverageMapper = p.Results.coverage_mapper;
            obj.RelayConfig = p.Results.relay_config;
            obj.MinRelayHeight = p.Results.MinRelayHeight;
            obj.MaxRelayHeight = p.Results.MaxRelayHeight;
            obj.TerrainData = coverage_mapper.TerrainData;
            obj.PropagationModel = coverage_mapper.PropagationModel;
        end
        
        function gap_analysis = analyzeGaps(obj, coverage_map, mission_area)
            % Analyze coverage gaps and identify dead zones
            % INPUTS:
            %   coverage_map: output from CoverageMapper.computeCoverage
            %   mission_area: [optional] Nx2 array of [x,y] waypoints defining mission area
            % OUTPUT:
            %   gap_analysis: struct with gap information
            
            fprintf('\n=== Analyzing Coverage Gaps ===\n');
            
            % Identify uncovered areas
            uncovered = ~coverage_map.Coverage;
            
            % Label connected regions (separate dead zones)
            CC = bwconncomp(uncovered);
            num_gaps = CC.NumObjects;
            
            fprintf('Found %d disconnected coverage gaps\n', num_gaps);
            
            % Analyze each gap
            gaps = struct('ID', {}, 'Area_m2', {}, 'Centroid', {}, ...
                         'BoundingBox', {}, 'Pixels', {}, 'Priority', {});
            
            for i = 1:num_gaps
                gap_pixels = CC.PixelIdxList{i};
                gap_area = length(gap_pixels) * obj.CoverageMapper.GridSpacing^2;
                
                % Get coordinates of gap pixels
                [rows, cols] = ind2sub(size(uncovered), gap_pixels);
                x_coords = coverage_map.X(gap_pixels);
                y_coords = coverage_map.Y(gap_pixels);
                
                % Centroid
                centroid = [mean(x_coords), mean(y_coords)];
                
                % Bounding box [min_x, max_x, min_y, max_y]
                bbox = [min(x_coords), max(x_coords), min(y_coords), max(y_coords)];
                
                gaps(i).ID = i;
                gaps(i).Area_m2 = gap_area;
                gaps(i).Centroid = centroid;
                gaps(i).BoundingBox = bbox;
                gaps(i).Pixels = gap_pixels;
                gaps(i).Priority = gap_area; % Default: prioritize by area
            end
            
            % If mission area specified, prioritize gaps that intersect it
            if nargin > 2 && ~isempty(mission_area)
                fprintf('Prioritizing gaps within mission area...\n');
                for i = 1:length(gaps)
                    % Check if gap overlaps mission area
                    x_coords = coverage_map.X(gaps(i).Pixels);
                    y_coords = coverage_map.Y(gaps(i).Pixels);
                    
                    % Create polygon for gap
                    gap_boundary = boundary(x_coords, y_coords, 1);
                    
                    % Check if any mission waypoints fall in gap
                    in_gap = inpolygon(mission_area(:,1), mission_area(:,2), ...
                                      x_coords(gap_boundary), y_coords(gap_boundary));
                    
                    if any(in_gap)
                        gaps(i).Priority = gaps(i).Priority * 10; % High priority
                        gaps(i).IntersectsMission = true;
                    else
                        gaps(i).IntersectsMission = false;
                    end
                end
            end
            
            % Sort gaps by priority
            [~, sort_idx] = sort([gaps.Priority], 'descend');
            gaps = gaps(sort_idx);
            
            % Print summary
            fprintf('\n--- Gap Analysis Summary ---\n');
            total_gap_area = sum([gaps.Area_m2]);
            fprintf('Total uncovered area: %.2f km² (%.1f%% of total)\n', ...
                    total_gap_area/1e6, 100 * sum(uncovered(:)) / numel(uncovered));
            
            fprintf('\nTop 5 Gaps:\n');
            for i = 1:min(5, length(gaps))
                fprintf('  Gap %d: %.3f km² at [%.1f, %.1f]', ...
                        gaps(i).ID, gaps(i).Area_m2/1e6, ...
                        gaps(i).Centroid(1), gaps(i).Centroid(2));
                if isfield(gaps(i), 'IntersectsMission') && gaps(i).IntersectsMission
                    fprintf(' [MISSION CRITICAL]');
                end
                fprintf('\n');
            end
            
            % Package results
            gap_analysis = struct();
            gap_analysis.Gaps = gaps;
            gap_analysis.NumGaps = num_gaps;
            gap_analysis.TotalGapArea_m2 = total_gap_area;
            gap_analysis.CoverageMap = coverage_map;
            gap_analysis.UncoveredMask = uncovered;
        end
        
        function relay_candidates = findRelayCandidates(obj, gap_analysis, base_station, num_candidates)
            % Find candidate locations for relay placement
            % INPUTS:
            %   gap_analysis: output from analyzeGaps
            %   base_station: AntennaConfig of base station to connect to
            %   num_candidates: number of candidate locations to return per gap
            % OUTPUT:
            %   relay_candidates: struct array with candidate locations
            
            if nargin < 4
                num_candidates = 5;
            end
            
            fprintf('\n=== Finding Relay Candidate Locations ===\n');
            
            coverage_map = gap_analysis.CoverageMap;
            gaps = gap_analysis.Gaps;
            
            all_candidates = [];
            candidate_id = 1;
            
            % For each gap, find relay locations
            for gap_idx = 1:min(3, length(gaps))  % Process top 3 gaps
                gap = gaps(gap_idx);
                
                fprintf('\nAnalyzing Gap %d (%.3f km²)...\n', ...
                        gap.ID, gap.Area_m2/1e6);
                
                % Get covered areas (potential relay locations)
                covered_mask = coverage_map.Coverage;
                
                % Find boundary between covered and uncovered near this gap
                % Look in expanded bounding box
                margin = 200; % meters
                x_min = max(gap.BoundingBox(1) - margin, min(coverage_map.X(:)));
                x_max = min(gap.BoundingBox(2) + margin, max(coverage_map.X(:)));
                y_min = max(gap.BoundingBox(3) - margin, min(coverage_map.Y(:)));
                y_max = min(gap.BoundingBox(4) + margin, max(coverage_map.Y(:)));
                
                % Find covered points near gap
                in_search_area = (coverage_map.X >= x_min & coverage_map.X <= x_max & ...
                                 coverage_map.Y >= y_min & coverage_map.Y <= y_max);
                
                candidate_mask = covered_mask & in_search_area;
                candidate_indices = find(candidate_mask);
                
                if isempty(candidate_indices)
                    fprintf('  No suitable relay locations found near gap\n');
                    continue;
                end
                
                % Score each candidate location
                scores = zeros(length(candidate_indices), 1);
                
                for i = 1:length(candidate_indices)
                    idx = candidate_indices(i);
                    x = coverage_map.X(idx);
                    y = coverage_map.Y(idx);
                    z_terrain = coverage_map.Z_terrain(idx);
                    
                    % Score based on multiple factors:
                    % 1. Distance to gap centroid (closer is better)
                    dist_to_gap = norm([x, y] - gap.Centroid);
                    score_distance = 1 / (1 + dist_to_gap/1000); % Normalize by km
                    
                    % 2. Received power from base station (stronger is better)
                    rx_power = coverage_map.ReceivedPower_dBm(idx);
                    score_power = (rx_power - obj.CoverageMapper.ReceiverSensitivity) / 50; % Normalize
                    score_power = max(0, min(1, score_power));
                    
                    % 3. Elevation (higher is generally better for relays)
                    z_range = max(coverage_map.Z_terrain(:)) - min(coverage_map.Z_terrain(:));
                    if z_range > 0
                        score_elevation = (z_terrain - min(coverage_map.Z_terrain(:))) / z_range;
                    else
                        score_elevation = 0.5;
                    end
                    
                    % 4. Line of sight to base station
                    score_los = double(coverage_map.LineOfSight(idx)) * 0.5;
                    
                    % Combined score
                    scores(i) = 0.4*score_distance + 0.3*score_power + ...
                               0.2*score_elevation + 0.1*score_los;
                end
                
                % Get top candidates
                [~, sort_idx] = sort(scores, 'descend');
                top_n = min(num_candidates, length(sort_idx));
                
                fprintf('  Found %d candidate locations\n', length(candidate_indices));
                
                for i = 1:top_n
                    idx = candidate_indices(sort_idx(i));
                    x = coverage_map.X(idx);
                    y = coverage_map.Y(idx);
                    z_terrain = coverage_map.Z_terrain(idx);
                    
                    % Suggest relay height (middle of allowed range)
                    relay_height = (obj.MinRelayHeight + obj.MaxRelayHeight) / 2;
                    
                    candidate = struct();
                    candidate.ID = candidate_id;
                    candidate.GapID = gap.ID;
                    candidate.Position = [x, y, z_terrain + relay_height];
                    candidate.TerrainElevation = z_terrain;
                    candidate.RelayHeight = relay_height;
                    candidate.Score = scores(sort_idx(i));
                    candidate.DistanceToGap_m = norm([x, y] - gap.Centroid);
                    candidate.RxPowerFromBase_dBm = coverage_map.ReceivedPower_dBm(idx);
                    candidate.LOStoBase = coverage_map.LineOfSight(idx);
                    
                    all_candidates = [all_candidates; candidate];
                    candidate_id = candidate_id + 1;
                    
                    fprintf('    Candidate %d: [%.1f, %.1f, %.1f] Score=%.2f, Dist=%.0fm, Rx=%.1fdBm\n', ...
                            candidate.ID, x, y, z_terrain + relay_height, ...
                            candidate.Score, candidate.DistanceToGap_m, ...
                            candidate.RxPowerFromBase_dBm);
                end
            end
            
            relay_candidates = all_candidates;
            
            fprintf('\nTotal relay candidates identified: %d\n', length(relay_candidates));
        end
        
        function [best_relay, improved_coverage] = evaluateRelayPlacement(obj, candidate, base_coverage)
            % Evaluate a specific relay candidate location
            % INPUTS:
            %   candidate: relay candidate struct
            %   base_coverage: existing coverage map without relay
            % OUTPUTS:
            %   best_relay: AntennaConfig for the relay
            %   improved_coverage: new coverage map with relay
            
            fprintf('\n=== Evaluating Relay Candidate %d ===\n', candidate.ID);
            
            % Create relay antenna config
            relay = obj.RelayConfig;
            relay.Position = candidate.Position;
            relay.Name = sprintf('Relay_%d', candidate.ID);
            
            % Compute coverage from relay
            relay_coverage = obj.CoverageMapper.computeCoverage(relay);
            
            % Combine with base coverage
            improved_coverage = obj.CoverageMapper.combineCoverage(...
                {base_coverage, relay_coverage});
            
            % Calculate improvement
            base_coverage_pct = base_coverage.Statistics.CoveragePercent;
            new_coverage_pct = improved_coverage.Statistics.CoveragePercent;
            improvement_pct = new_coverage_pct - base_coverage_pct;
            
            fprintf('Coverage improvement: %.1f%% -> %.1f%% (+%.1f%%)\n', ...
                    base_coverage_pct, new_coverage_pct, improvement_pct);
            
            best_relay = relay;
        end
        
        function fig = visualizeGapsAndCandidates(obj, gap_analysis, relay_candidates)
            % Visualize coverage gaps and relay candidates
            
            fig = figure('Position', [100, 100, 1400, 800]);
            
            coverage_map = gap_analysis.CoverageMap;
            
            % Plot coverage
            P_plot = coverage_map.ReceivedPower_dBm;
            P_plot(~isfinite(P_plot)) = obj.CoverageMapper.ReceiverSensitivity - 20;
            
            imagesc(coverage_map.X(1,:), coverage_map.Y(:,1), P_plot);
            hold on;
            
            % Colormap
            n_colors = 256;
            cmap = [linspace(1,0,n_colors)', linspace(0,1,n_colors)', zeros(n_colors,1)];
            colormap(cmap);
            colorbar;
            caxis([obj.CoverageMapper.ReceiverSensitivity, max(P_plot(:))]);
            
            % Highlight gaps
            for i = 1:length(gap_analysis.Gaps)
                gap = gap_analysis.Gaps(i);
                x_coords = coverage_map.X(gap.Pixels);
                y_coords = coverage_map.Y(gap.Pixels);
                
                % Draw gap boundary
                if length(x_coords) > 3
                    k = boundary(x_coords, y_coords, 0.8);
                    plot(x_coords(k), y_coords(k), 'r-', 'LineWidth', 2);
                    
                    % Label gap
                    text(gap.Centroid(1), gap.Centroid(2), ...
                         sprintf('Gap %d', gap.ID), ...
                         'Color', 'white', 'FontWeight', 'bold', ...
                         'BackgroundColor', 'red', 'FontSize', 10);
                end
            end
            
            % Plot base station
            if ~isempty(coverage_map.AntennaConfig)
                base_pos = coverage_map.AntennaConfig.Position;
                plot(base_pos(1), base_pos(2), 'k^', 'MarkerSize', 20, ...
                     'MarkerFaceColor', 'yellow', 'LineWidth', 2);
                text(base_pos(1), base_pos(2) + 50, 'BASE', ...
                     'Color', 'yellow', 'FontWeight', 'bold', 'FontSize', 12);
            end
            
            % Plot relay candidates
            if nargin > 2 && ~isempty(relay_candidates)
                for i = 1:length(relay_candidates)
                    cand = relay_candidates(i);
                    
                    % Color by score (green = best)
                    color_val = cand.Score;
                    marker_color = [1-color_val, color_val, 0];
                    
                    plot(cand.Position(1), cand.Position(2), 'o', ...
                         'MarkerSize', 12, 'MarkerFaceColor', marker_color, ...
                         'MarkerEdgeColor', 'black', 'LineWidth', 2);
                    
                    text(cand.Position(1) + 30, cand.Position(2), ...
                         sprintf('R%d', cand.ID), ...
                         'Color', 'white', 'FontWeight', 'bold', 'FontSize', 9);
                end
            end
            
            xlabel('Easting (m)');
            ylabel('Northing (m)');
            title('Coverage Gaps and Relay Candidates');
            axis equal tight;
            set(gca, 'YDir', 'normal');
            grid on;
            
            % Legend
            legend_items = {};
            if ~isempty(coverage_map.AntennaConfig)
                legend_items{end+1} = 'Base Station';
            end
            if nargin > 2 && ~isempty(relay_candidates)
                legend_items{end+1} = 'Relay Candidates';
            end
            legend_items{end+1} = 'Coverage Gaps';
            
            if ~isempty(legend_items)
                legend(legend_items, 'Location', 'best');
            end
        end
        
        function report = generateRelayReport(obj, gap_analysis, relay_candidates, mission_area)
            % Generate comprehensive relay placement report
            
            fprintf('\n');
            fprintf('========================================================\n');
            fprintf('           RELAY PLACEMENT RECOMMENDATION REPORT        \n');
            fprintf('========================================================\n\n');
            
            % Coverage statistics
            fprintf('CURRENT COVERAGE STATISTICS:\n');
            fprintf('  Total area: %.3f km²\n', gap_analysis.CoverageMap.Statistics.TotalArea_km2);
            fprintf('  Covered: %.3f km² (%.1f%%)\n', ...
                    gap_analysis.CoverageMap.Statistics.CoveredArea_km2, ...
                    gap_analysis.CoverageMap.Statistics.CoveragePercent);
            fprintf('  Uncovered: %.3f km² (%.1f%%)\n', ...
                    gap_analysis.TotalGapArea_m2/1e6, ...
                    100 - gap_analysis.CoverageMap.Statistics.CoveragePercent);
            fprintf('  Number of gaps: %d\n', gap_analysis.NumGaps);
            
            % Mission area analysis
            if nargin > 3 && ~isempty(mission_area)
                fprintf('\nMISSION AREA ANALYSIS:\n');
                % Check coverage at mission waypoints
                covered_waypoints = 0;
                for i = 1:size(mission_area, 1)
                    x = mission_area(i, 1);
                    y = mission_area(i, 2);
                    % Interpolate coverage
                    is_covered = interp2(gap_analysis.CoverageMap.X, ...
                                        gap_analysis.CoverageMap.Y, ...
                                        double(gap_analysis.CoverageMap.Coverage), ...
                                        x, y, 'nearest');
                    if is_covered
                        covered_waypoints = covered_waypoints + 1;
                    end
                end
                fprintf('  Mission waypoints covered: %d / %d (%.1f%%)\n', ...
                        covered_waypoints, size(mission_area, 1), ...
                        100*covered_waypoints/size(mission_area, 1));
            end
            
            % Top relay recommendations
            fprintf('\nTOP RELAY PLACEMENT RECOMMENDATIONS:\n');
            fprintf('  (Sorted by score - higher is better)\n\n');
            
            for i = 1:min(10, length(relay_candidates))
                cand = relay_candidates(i);
                fprintf('  %d. RELAY CANDIDATE %d (Gap %d)\n', i, cand.ID, cand.GapID);
                fprintf('     Position: [%.1f E, %.1f N, %.1f m ASL]\n', ...
                        cand.Position(1), cand.Position(2), cand.Position(3));
                fprintf('     Height above ground: %.1f m\n', cand.RelayHeight);
                fprintf('     Score: %.3f\n', cand.Score);
                fprintf('     Distance to gap: %.0f m\n', cand.DistanceToGap_m);
                fprintf('     Rx power from base: %.1f dBm\n', cand.RxPowerFromBase_dBm);
                fprintf('     LOS to base: %s\n', char(string(cand.LOStoBase)));
                fprintf('\n');
            end
            
            fprintf('========================================================\n\n');
            
            % Package report
            report = struct();
            report.GapAnalysis = gap_analysis;
            report.RelayCandidates = relay_candidates;
            report.Timestamp = datetime('now');
        end
    end
end