classdef CoverageMapper
    % COVERAGEMAPPER Main class for computing and visualizing antenna coverage
    % Handles coverage computation and visualization for antenna configurations
    
    properties
        TerrainData         % TerrainData object
        PropagationModel    % RayTracingPropagation object
        ReceiverHeight      % Height of receivers above ground (m)
        ReceiverSensitivity % Minimum received power in dBm
        GridSpacing         % Coverage grid spacing in meters
    end
    
    methods
        function obj = CoverageMapper(terrain_data, propagation_model, varargin)
            % Constructor
            p = inputParser;
            addRequired(p, 'terrain_data', @(x) isa(x, 'TerrainData'));
            addRequired(p, 'propagation_model', @(x) isa(x, 'RayTracingPropagation'));
            addParameter(p, 'ReceiverHeight', 2.0, @isnumeric); % 2m above ground
            addParameter(p, 'ReceiverSensitivity', -100, @isnumeric); % dBm
            addParameter(p, 'GridSpacing', 20, @isnumeric); % meters
            
            parse(p, terrain_data, propagation_model, varargin{:});
            
            obj.TerrainData = p.Results.terrain_data;
            obj.PropagationModel = p.Results.propagation_model;
            obj.ReceiverHeight = p.Results.ReceiverHeight;
            obj.ReceiverSensitivity = p.Results.ReceiverSensitivity;
            obj.GridSpacing = p.Results.GridSpacing;
        end
        
        function coverage_map = computeCoverage(obj, antenna_config)
            % Compute coverage map for a given antenna configuration
            % INPUT:
            %   antenna_config: AntennaConfig object
            % OUTPUT:
            %   coverage_map: struct with coverage data
            
            % Set terrain position if using Location+HeightAGL mode
            if ~isempty(antenna_config.Location) && isempty(antenna_config.TerrainElevation)
                antenna_config = antenna_config.setTerrainPosition(obj.TerrainData);
            end
            
            fprintf('\n=== Computing Coverage for %s ===\n', antenna_config.Name);
            antenna_config.display();
            
            % Create receiver grid
            x_min = obj.TerrainData.Limits(1);
            x_max = obj.TerrainData.Limits(2);
            y_min = obj.TerrainData.Limits(3);
            y_max = obj.TerrainData.Limits(4);
            
            x_grid = x_min:obj.GridSpacing:x_max;
            y_grid = y_min:obj.GridSpacing:y_max;
            [X_grid, Y_grid] = meshgrid(x_grid, y_grid);
            
            % Get terrain elevation at grid points
            Z_terrain_grid = obj.TerrainData.getElevation(X_grid, Y_grid);
            Z_grid = Z_terrain_grid + obj.ReceiverHeight;
            
            % Flatten grids for vectorized computation
            num_points = numel(X_grid);
            rx_positions = [X_grid(:), Y_grid(:), Z_grid(:)];
            
            fprintf('Computing coverage for %d receiver points...\n', num_points);
            tic;
            
            % Compute path loss for all points
            tx_pos = antenna_config.Position;
            frequency = antenna_config.Frequency;
            
            [path_loss_dB, LOS_flag] = obj.PropagationModel.computePathLoss(...
                tx_pos, rx_positions, frequency);
            
            % Compute received power
            % P_rx (dBm) = P_tx (dBm) + G_tx (dBi) - Path Loss (dB) + G_rx (dBi) - Misc Losses
            P_tx_dBm = 10*log10(antenna_config.Power * 1000); % Convert W to dBm
            G_tx_dBi = antenna_config.Gain;
            G_rx_dBi = 0; % Assuming isotropic receiver
            misc_losses_dB = 3; % Cable losses, etc.
            
            % Apply antenna pattern
            pattern_gain_dB = obj.computeAntennaPattern(antenna_config, rx_positions);
            
            % Total received power
            P_rx_dBm = P_tx_dBm + G_tx_dBi + pattern_gain_dB - path_loss_dB + G_rx_dBi - misc_losses_dB;
            
            % Reshape results
            P_rx_grid = reshape(P_rx_dBm, size(X_grid));
            LOS_grid = reshape(LOS_flag, size(X_grid));
            PathLoss_grid = reshape(path_loss_dB, size(X_grid));
            
            % Mark out-of-range points
            P_rx_grid(isinf(PathLoss_grid)) = -inf;
            
            % Coverage indicator (above sensitivity)
            Coverage_grid = P_rx_grid >= obj.ReceiverSensitivity;
            
            elapsed = toc;
            fprintf('Coverage computation completed in %.2f seconds\n', elapsed);
            
            % Calculate statistics
            total_area = numel(Coverage_grid) * obj.GridSpacing^2 / 1e6; % km^2
            covered_area = sum(Coverage_grid(:)) * obj.GridSpacing^2 / 1e6; % km^2
            coverage_percent = 100 * sum(Coverage_grid(:)) / numel(Coverage_grid);
            los_percent = 100 * sum(LOS_grid(:)) / numel(LOS_grid);
            
            fprintf('\n--- Coverage Statistics ---\n');
            fprintf('Total area: %.3f km²\n', total_area);
            fprintf('Covered area: %.3f km² (%.1f%%)\n', covered_area, coverage_percent);
            fprintf('Line-of-sight: %.1f%% of grid points\n', los_percent);
            fprintf('Average received power: %.2f dBm\n', mean(P_rx_grid(isfinite(P_rx_grid))));
            fprintf('Max received power: %.2f dBm\n', max(P_rx_grid(isfinite(P_rx_grid))));
            fprintf('Min received power: %.2f dBm\n', min(P_rx_grid(isfinite(P_rx_grid))));
            
            % Package results
            coverage_map = struct();
            coverage_map.X = X_grid;
            coverage_map.Y = Y_grid;
            coverage_map.Z_terrain = Z_terrain_grid;
            coverage_map.Z_receiver = Z_grid;
            coverage_map.ReceivedPower_dBm = P_rx_grid;
            coverage_map.PathLoss_dB = PathLoss_grid;
            coverage_map.Coverage = Coverage_grid;
            coverage_map.LineOfSight = LOS_grid;
            coverage_map.AntennaConfig = antenna_config;
            coverage_map.ReceiverSensitivity_dBm = obj.ReceiverSensitivity;
            coverage_map.Statistics = struct('TotalArea_km2', total_area, ...
                                             'CoveredArea_km2', covered_area, ...
                                             'CoveragePercent', coverage_percent, ...
                                             'LOSPercent', los_percent);
        end
        
        function pattern_gain_dB = computeAntennaPattern(obj, antenna_config, rx_positions)
            % Compute antenna pattern gain for receiver positions
            % Simplified pattern model
            
            tx_pos = antenna_config.Position;
            num_rx = size(rx_positions, 1);
            pattern_gain_dB = zeros(num_rx, 1);
            
            % Vector from TX to RX
            dx = rx_positions(:, 1) - tx_pos(1);
            dy = rx_positions(:, 2) - tx_pos(2);
            dz = rx_positions(:, 3) - tx_pos(3);
            
            % Azimuth angle (0° = North, 90° = East)
            azimuth_to_rx = atan2d(dx, dy); % Note: atan2d(x,y) for N=0°
            azimuth_to_rx = mod(azimuth_to_rx, 360);
            
            % Elevation angle
            dist_horizontal = sqrt(dx.^2 + dy.^2);
            elevation_to_rx = atan2d(dz, dist_horizontal);
            
            % Angle difference from antenna boresight
            azimuth_diff = abs(azimuth_to_rx - antenna_config.Azimuth);
            azimuth_diff = min(azimuth_diff, 360 - azimuth_diff); % Shortest angle
            
            elevation_diff = abs(elevation_to_rx - antenna_config.Elevation);
            
            % Pattern type
            switch lower(antenna_config.PatternType)
                case 'omnidirectional'
                    % Uniform in azimuth, cosine pattern in elevation
                    pattern_gain_dB = -min(12, (elevation_diff / 30).^2 * 12);
                    
                case 'directional'
                    % Gaussian-like beam pattern
                    az_bw = antenna_config.BeamWidth(1);
                    el_bw = antenna_config.BeamWidth(2);
                    
                    % -3dB at half-power beamwidth
                    az_factor = (azimuth_diff / (az_bw/2)).^2;
                    el_factor = (elevation_diff / (el_bw/2)).^2;
                    
                    pattern_gain_dB = -3 * (az_factor + el_factor);
                    pattern_gain_dB = max(pattern_gain_dB, -30); % Front-to-back ratio
                    
                otherwise
                    % Default: omnidirectional
                    pattern_gain_dB = zeros(num_rx, 1);
            end
        end
        
        function fig = visualizeCoverage(obj, coverage_map, options)
            % Visualize coverage map
            % INPUT:
            %   coverage_map: output from computeCoverage
            %   options: struct with visualization options
            
            if nargin < 3
                options = struct();
            end
            
            % Default options
            if ~isfield(options, 'ShowTerrain'), options.ShowTerrain = true; end
            if ~isfield(options, 'Show3D'), options.Show3D = false; end
            if ~isfield(options, 'ShowLOS'), options.ShowLOS = false; end
            if ~isfield(options, 'ColorLimits'), options.ColorLimits = []; end
            if ~isfield(options, 'SavePath'), options.SavePath = ''; end
            
            fig = figure('Position', [100, 100, 1200, 900]);
            
            if options.Show3D
                % 3D visualization with terrain
                surf(coverage_map.X, coverage_map.Y, coverage_map.Z_terrain, ...
                     coverage_map.ReceivedPower_dBm, 'EdgeColor', 'none');
                hold on;
                
                % Plot antenna (if single antenna)
                if ~isempty(coverage_map.AntennaConfig)
                    tx_pos = coverage_map.AntennaConfig.Position;
                    plot3(tx_pos(1), tx_pos(2), tx_pos(3), 'r^', ...
                          'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
                end
                
                xlabel('Easting (m)');
                ylabel('Northing (m)');
                zlabel('Elevation (m)');
                view(45, 30);
                axis tight;
                lighting gouraud;
                camlight;
                
            else
                % 2D coverage overlay
                if options.ShowLOS
                    % Show LOS vs NLOS
                    imagesc(coverage_map.X(1,:), coverage_map.Y(:,1), ...
                            double(coverage_map.LineOfSight));
                    colormap(gray);
                    cbar = colorbar;
                    cbar.Label.String = 'Line of Sight (1=Yes, 0=No)';
                else
                    % Show received power
                    P_plot = coverage_map.ReceivedPower_dBm;
                    P_plot(~isfinite(P_plot)) = obj.ReceiverSensitivity - 20;
                    
                    imagesc(coverage_map.X(1,:), coverage_map.Y(:,1), P_plot);
                    
                    % Custom colormap: red (weak) to green (strong)
                    n_colors = 256;
                    cmap = [linspace(1,0,n_colors)', linspace(0,1,n_colors)', zeros(n_colors,1)];
                    colormap(cmap);
                    
                    cbar = colorbar;
                    cbar.Label.String = 'Received Power (dBm)';
                    
                    if ~isempty(options.ColorLimits)
                        caxis(options.ColorLimits);
                    else
                        caxis([obj.ReceiverSensitivity, max(P_plot(:))]);
                    end
                end
                
                hold on;
                
                % Overlay terrain contours if requested
                if options.ShowTerrain
                    % Check if terrain has variation before contouring
                    terrain_range = max(coverage_map.Z_terrain(:)) - min(coverage_map.Z_terrain(:));
                    terrain_has_variation = terrain_range > 0.1 && ~any(isnan(coverage_map.Z_terrain(:)));
                    
                    if terrain_has_variation
                        try
                            contour(coverage_map.X, coverage_map.Y, coverage_map.Z_terrain, ...
                                    10, 'k', 'LineWidth', 0.5, 'LineStyle', '--');
                        catch ME
                            % Silently skip contours if they fail
                            % (e.g., for constant or invalid terrain data)
                        end
                    end
                end
                
                % Plot antenna location (if single antenna)
                if ~isempty(coverage_map.AntennaConfig)
                    tx_pos = coverage_map.AntennaConfig.Position;
                    plot(tx_pos(1), tx_pos(2), 'r^', 'MarkerSize', 15, ...
                         'MarkerFaceColor', 'r', 'LineWidth', 2);
                    text(tx_pos(1), tx_pos(2) + 50, coverage_map.AntennaConfig.Name, ...
                         'Color', 'red', 'FontWeight', 'bold', 'FontSize', 12);
                end
                
                % Draw coverage boundary
                contour(coverage_map.X, coverage_map.Y, ...
                        double(coverage_map.Coverage), [0.5, 0.5], ...
                        'r', 'LineWidth', 2);
                
                xlabel('Easting (m)');
                ylabel('Northing (m)');
                axis equal tight;
                set(gca, 'YDir', 'normal');
            end
            
            % Create appropriate title
            if ~isempty(coverage_map.AntennaConfig)
                title_str = sprintf('Coverage Map: %s (%.1f MHz, %.1f dBm Sensitivity)', ...
                      coverage_map.AntennaConfig.Name, ...
                      coverage_map.AntennaConfig.Frequency/1e6, ...
                      obj.ReceiverSensitivity);
            else
                title_str = sprintf('Combined Coverage Map (%.1f dBm Sensitivity)', ...
                      obj.ReceiverSensitivity);
            end
            title(title_str);
            
            grid on;
            
            % Add statistics text box
            if isfield(coverage_map.Statistics, 'LOSPercent')
                stats_text = sprintf(['Coverage: %.1f%%\n', ...
                                     'LOS: %.1f%%\n', ...
                                     'Area: %.3f km²'], ...
                                    coverage_map.Statistics.CoveragePercent, ...
                                    coverage_map.Statistics.LOSPercent, ...
                                    coverage_map.Statistics.CoveredArea_km2);
            else
                stats_text = sprintf(['Coverage: %.1f%%\n', ...
                                     'Area: %.3f km²'], ...
                                    coverage_map.Statistics.CoveragePercent, ...
                                    coverage_map.Statistics.CoveredArea_km2);
            end
            
            annotation('textbox', [0.15, 0.15, 0.2, 0.1], 'String', stats_text, ...
                      'FitBoxToText', 'on', 'BackgroundColor', 'white', ...
                      'EdgeColor', 'black', 'FontSize', 10);
            
            % Save if path provided
            if ~isempty(options.SavePath)
                saveas(fig, options.SavePath);
                fprintf('Figure saved to: %s\n', options.SavePath);
            end
        end
        
        function combined_map = combineCoverage(obj, coverage_maps)
            % Combine multiple coverage maps (e.g., from multiple antennas)
            % Takes the maximum received power at each point
            
            fprintf('\n=== Combining %d Coverage Maps ===\n', length(coverage_maps));
            
            % Initialize with first map
            combined_map = coverage_maps{1};
            combined_map.AntennaConfig = []; % Multiple antennas
            
            % Combine received power (take maximum)
            for i = 2:length(coverage_maps)
                combined_map.ReceivedPower_dBm = max(combined_map.ReceivedPower_dBm, ...
                                                     coverage_maps{i}.ReceivedPower_dBm);
                combined_map.LineOfSight = combined_map.LineOfSight | ...
                                          coverage_maps{i}.LineOfSight;
            end
            
            % Recompute coverage
            combined_map.Coverage = combined_map.ReceivedPower_dBm >= obj.ReceiverSensitivity;
            
            % Update statistics
            total_area = numel(combined_map.Coverage) * obj.GridSpacing^2 / 1e6;
            covered_area = sum(combined_map.Coverage(:)) * obj.GridSpacing^2 / 1e6;
            coverage_percent = 100 * sum(combined_map.Coverage(:)) / numel(combined_map.Coverage);
            los_percent = 100 * sum(combined_map.LineOfSight(:)) / numel(combined_map.LineOfSight);
            
            combined_map.Statistics = struct('TotalArea_km2', total_area, ...
                                            'CoveredArea_km2', covered_area, ...
                                            'CoveragePercent', coverage_percent, ...
                                            'LOSPercent', los_percent);
            
            fprintf('Combined coverage: %.1f%% (%.3f km²)\n', ...
                    coverage_percent, covered_area);
            fprintf('Combined LOS: %.1f%%\n', los_percent);
        end
        
        function exportToGeoTIFF(obj, coverage_map, filename)
            % Export coverage map to GeoTIFF
            
            fprintf('Exporting coverage map to: %s\n', filename);
            
            % Create spatial referencing
            R = obj.TerrainData.R;
            R_coverage = maprefcells([min(coverage_map.Y(:)), max(coverage_map.Y(:))], ...
                                     [min(coverage_map.X(:)), max(coverage_map.X(:))], ...
                                     size(coverage_map.ReceivedPower_dBm));
            
            % Write to GeoTIFF
            geotiffwrite(filename, coverage_map.ReceivedPower_dBm, R_coverage);
            
            fprintf('Export complete.\n');
        end
    end
end