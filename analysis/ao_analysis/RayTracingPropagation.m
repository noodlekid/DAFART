classdef RayTracingPropagation
    % RAYTRACINGPROPAGATION Ray tracing propagation model for NLOS coverage
    % Implements ray tracing with terrain interaction for wireless propagation
    
    properties
        TerrainData         % TerrainData object
        MaxRange            % Maximum propagation range in meters
        RayResolution       % Resolution for ray tracing in meters
        NumReflections      % Maximum number of reflections to consider
        GroundPermittivity  % Relative permittivity of ground (Utah desert ~3-5)
        GroundConductivity  % Ground conductivity in S/m (desert ~0.001-0.01)
        AtmosphericLoss     % Atmospheric loss in dB/km
    end
    
    methods
        function obj = RayTracingPropagation(terrain_data, varargin)
            % Constructor
            p = inputParser;
            addRequired(p, 'terrain_data', @(x) isa(x, 'TerrainData'));
            addParameter(p, 'MaxRange', 5000, @isnumeric);
            addParameter(p, 'RayResolution', 10, @isnumeric);
            addParameter(p, 'NumReflections', 2, @isnumeric);
            addParameter(p, 'GroundPermittivity', 4, @isnumeric); % Utah desert
            addParameter(p, 'GroundConductivity', 0.005, @isnumeric); % S/m
            addParameter(p, 'AtmosphericLoss', 0.1, @isnumeric); % dB/km
            
            parse(p, terrain_data, varargin{:});
            
            obj.TerrainData = p.Results.terrain_data;
            obj.MaxRange = p.Results.MaxRange;
            obj.RayResolution = p.Results.RayResolution;
            obj.NumReflections = p.Results.NumReflections;
            obj.GroundPermittivity = p.Results.GroundPermittivity;
            obj.GroundConductivity = p.Results.GroundConductivity;
            obj.AtmosphericLoss = p.Results.AtmosphericLoss;
        end
        
        function [path_loss_dB, LOS_flag] = computePathLoss(obj, tx_pos, rx_pos, frequency)
            % Compute path loss between transmitter and receiver
            % INPUTS:
            %   tx_pos: [x, y, z] transmitter position in meters
            %   rx_pos: [x, y, z] receiver position(s) - can be Nx3 matrix
            %   frequency: operating frequency in Hz
            % OUTPUTS:
            %   path_loss_dB: path loss in dB
            %   LOS_flag: logical array indicating line-of-sight
            
            % Handle single or multiple receiver positions
            if size(rx_pos, 1) == 1
                rx_pos = rx_pos(:)';
            end
            
            num_rx = size(rx_pos, 1);
            path_loss_dB = zeros(num_rx, 1);
            LOS_flag = false(num_rx, 1);
            
            wavelength = 3e8 / frequency;
            
            for i = 1:num_rx
                % Check line of sight
                [is_los, clearance] = obj.checkLineOfSight(tx_pos, rx_pos(i, :));
                LOS_flag(i) = is_los;
                
                % Calculate distance
                distance = norm(tx_pos - rx_pos(i, :));
                
                if distance > obj.MaxRange
                    path_loss_dB(i) = inf;
                    continue;
                end
                
                % Free space path loss
                if distance < 1
                    distance = 1; % Avoid singularity
                end
                FSPL = 20*log10(distance) + 20*log10(frequency) - 147.55;
                
                if is_los
                    % Line of sight with Fresnel zone consideration
                    path_loss_dB(i) = FSPL + obj.getFresnelLoss(clearance, distance, wavelength);
                else
                    % NLOS - use diffraction and reflection model
                    path_loss_dB(i) = obj.computeNLOSPathLoss(tx_pos, rx_pos(i, :), ...
                                                               frequency, FSPL);
                end
                
                % Add atmospheric loss
                path_loss_dB(i) = path_loss_dB(i) + obj.AtmosphericLoss * distance / 1000;
            end
        end
        
        function [is_los, min_clearance] = checkLineOfSight(obj, pos1, pos2)
            % Check if there is line of sight between two points
            % INPUTS:
            %   pos1, pos2: [x, y, z] positions in meters
            % OUTPUTS:
            %   is_los: true if line of sight exists
            %   min_clearance: minimum clearance above terrain in meters
            
            % Create ray from pos1 to pos2
            num_points = ceil(norm(pos1 - pos2) / obj.RayResolution);
            if num_points < 2
                num_points = 2;
            end
            
            x_ray = linspace(pos1(1), pos2(1), num_points);
            y_ray = linspace(pos1(2), pos2(2), num_points);
            z_ray = linspace(pos1(3), pos2(3), num_points);
            
            % Get terrain elevation along ray
            z_terrain = obj.TerrainData.getElevation(x_ray, y_ray);
            
            % Calculate clearance
            clearance = z_ray - z_terrain;
            
            % Check if any point intersects terrain (with small margin)
            margin = 1.0; % 1 meter clearance required
            is_los = all(clearance > margin);
            min_clearance = min(clearance);
        end
        
        function fresnel_loss = getFresnelLoss(obj, clearance, distance, wavelength)
            % Calculate loss due to Fresnel zone obstruction
            % INPUTS:
            %   clearance: minimum clearance in meters
            %   distance: total path distance in meters
            %   wavelength: wavelength in meters
            % OUTPUT:
            %   fresnel_loss: additional loss in dB
            
            % First Fresnel zone radius at midpoint
            fresnel_radius = sqrt(wavelength * distance / 4);
            
            if clearance >= 0.6 * fresnel_radius
                % Good clearance
                fresnel_loss = 0;
            elseif clearance > 0
                % Partial obstruction
                v = clearance / fresnel_radius * sqrt(2);
                fresnel_loss = 6.9 + 20*log10(sqrt((v-0.1)^2 + 1) + v - 0.1);
            else
                % Complete obstruction - NLOS
                fresnel_loss = 20; % Will be handled by NLOS model
            end
            
            fresnel_loss = max(0, fresnel_loss);
        end
        
        function nlos_loss = computeNLOSPathLoss(obj, tx_pos, rx_pos, frequency, fspl)
            % Compute NLOS path loss using diffraction and reflection
            % This is a simplified model for desert terrain
            
            % Two-ray ground reflection model adapted for terrain
            distance_3d = norm(tx_pos - rx_pos);
            distance_2d = norm(tx_pos(1:2) - rx_pos(1:2));
            
            % Get average terrain elevation between points
            num_samples = 20;
            x_samples = linspace(tx_pos(1), rx_pos(1), num_samples);
            y_samples = linspace(tx_pos(2), rx_pos(2), num_samples);
            z_terrain_avg = mean(obj.TerrainData.getElevation(x_samples, y_samples), 'omitnan');
            
            % Effective heights above average terrain
            h_tx = tx_pos(3) - z_terrain_avg;
            h_rx = rx_pos(3) - z_terrain_avg;
            
            if h_tx < 0
                h_tx = 1;
            end
            if h_rx < 0
                h_rx = 1;
            end
            
            % Ground reflection coefficient (Fresnel coefficient)
            wavelength = 3e8 / frequency;
            grazing_angle = atan((h_tx + h_rx) / distance_2d);
            
            % Simplified reflection coefficient for desert terrain
            R = obj.computeReflectionCoefficient(grazing_angle, frequency);
            
            % Path difference for ground reflection
            d_direct = distance_3d;
            d_reflected = sqrt(distance_2d^2 + (h_tx + h_rx)^2);
            path_diff = d_reflected - d_direct;
            
            % Phase difference
            phase_diff = 2 * pi * path_diff / wavelength;
            
            % Two-ray model
            E_direct = 1 / d_direct;
            E_reflected = R / d_reflected * exp(1i * phase_diff);
            E_total = abs(E_direct + E_reflected);
            
            % Convert to path loss
            if E_total > 0
                two_ray_loss = -20*log10(E_total * distance_3d);
            else
                two_ray_loss = 40; % High loss if destructive interference
            end
            
            % Use the worse of FSPL + margin or two-ray model
            nlos_margin = 15; % Additional NLOS margin in dB
            nlos_loss = max(fspl + nlos_margin, two_ray_loss);
            
            % Add diffraction loss for significant obstacles
            nlos_loss = nlos_loss + obj.estimateDiffractionLoss(tx_pos, rx_pos);
        end
        
        function R = computeReflectionCoefficient(obj, grazing_angle, frequency)
            % Compute Fresnel reflection coefficient for ground
            % Simplified for vertical polarization
            
            epsilon_r = obj.GroundPermittivity;
            sigma = obj.GroundConductivity;
            epsilon_0 = 8.854e-12;
            omega = 2 * pi * frequency;
            
            % Complex permittivity
            epsilon_c = epsilon_r - 1i * sigma / (omega * epsilon_0);
            
            % Reflection coefficient for vertical polarization
            sin_theta = sin(grazing_angle);
            cos_theta = cos(grazing_angle);
            
            sqrt_term = sqrt(epsilon_c - cos_theta^2);
            numerator = epsilon_c * sin_theta - sqrt_term;
            denominator = epsilon_c * sin_theta + sqrt_term;
            
            R = abs(numerator / denominator);
            R = min(R, 1.0); % Ensure physical bounds
        end
        
        function diff_loss = estimateDiffractionLoss(obj, tx_pos, rx_pos)
            % Estimate additional diffraction loss over terrain obstacles
            % Simplified knife-edge diffraction model
            
            % Sample terrain profile
            num_samples = 50;
            x_profile = linspace(tx_pos(1), rx_pos(1), num_samples);
            y_profile = linspace(tx_pos(2), rx_pos(2), num_samples);
            z_profile = obj.TerrainData.getElevation(x_profile, y_profile);
            
            % Line connecting tx and rx
            z_line = linspace(tx_pos(3), rx_pos(3), num_samples);
            
            % Find maximum obstruction
            h_diff = z_profile - z_line;
            h_max = max(h_diff);
            
            if h_max <= 0
                diff_loss = 0;
                return;
            end
            
            % Simplified knife-edge diffraction parameter
            distance = norm(tx_pos - rx_pos);
            v = h_max * sqrt(2 / (3e8/915e6) / distance); % Approximation
            
            % Diffraction loss
            if v > -0.8
                diff_loss = 6.9 + 20*log10(sqrt((v-0.1)^2 + 1) + v - 0.1);
            else
                diff_loss = 0;
            end
            
            diff_loss = max(0, min(diff_loss, 25)); % Limit to reasonable range
        end
    end
end