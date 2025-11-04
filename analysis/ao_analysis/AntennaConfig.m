classdef AntennaConfig
    % ANTENNACONFIG Configuration class for antenna parameters
    % Stores all antenna-specific parameters for coverage analysis
    %
    % USAGE:
    %   Method 1 (RECOMMENDED): Specify location + height above ground
    %       antenna = AntennaConfig('Location', [x, y], 'HeightAGL', 10, ...)
    %       Position will be set automatically when used with terrain data
    %
    %   Method 2: Specify absolute position (legacy)
    %       antenna = AntennaConfig('Position', [x, y, z_msl], ...)
    
    properties
        Position            % [x, y, z] in meters (UTM easting, northing, altitude MSL)
        Location            % [x, y] in meters (UTM easting, northing) - if set, Position(3) is calculated
        HeightAGL           % Height above ground level in meters
        Frequency           % Operating frequency in Hz
        Power               % Transmit power in Watts
        Gain                % Antenna gain in dBi
        PatternType         % 'omnidirectional', 'dipole', 'patch', 'custom'
        Polarization        % 'vertical', 'horizontal', 'circular'
        Azimuth             % Azimuth angle in degrees (0-360)
        Elevation           % Elevation tilt angle in degrees (-90 to 90)
        BeamWidth           % [azimuth_beamwidth, elevation_beamwidth] in degrees
        Name                % Identifier for this antenna
        TerrainElevation    % Terrain elevation at antenna location (set automatically)
    end
    
    methods
        function obj = AntennaConfig(varargin)
            % Constructor with default values
            p = inputParser;
            addParameter(p, 'Position', [], @(x) isempty(x) || (isnumeric(x) && length(x) == 3));
            addParameter(p, 'Location', [], @(x) isempty(x) || (isnumeric(x) && length(x) == 2));
            addParameter(p, 'HeightAGL', 10, @isnumeric); % 10m above ground default
            addParameter(p, 'Frequency', 915e6, @isnumeric); % 915 MHz default
            addParameter(p, 'Power', 1, @isnumeric); % 1 Watt
            addParameter(p, 'Gain', 0, @isnumeric); % 0 dBi (isotropic)
            addParameter(p, 'PatternType', 'omnidirectional', @ischar);
            addParameter(p, 'Polarization', 'vertical', @ischar);
            addParameter(p, 'Azimuth', 0, @isnumeric);
            addParameter(p, 'Elevation', 0, @isnumeric);
            addParameter(p, 'BeamWidth', [360, 90], @(x) isnumeric(x) && length(x) == 2);
            addParameter(p, 'Name', 'Antenna_1', @ischar);
            
            parse(p, varargin{:});
            
            % Handle Position vs Location+HeightAGL
            if ~isempty(p.Results.Position)
                % Legacy mode: absolute position specified
                obj.Position = p.Results.Position;
                obj.Location = p.Results.Position(1:2);
                obj.HeightAGL = [];  % Unknown
                obj.TerrainElevation = [];
            elseif ~isempty(p.Results.Location)
                % New mode: location + height above ground
                obj.Location = p.Results.Location;
                obj.HeightAGL = p.Results.HeightAGL;
                obj.Position = [obj.Location, 0];  % Z will be set when terrain is available
                obj.TerrainElevation = [];
            else
                % No location specified - default
                obj.Location = [0, 0];
                obj.HeightAGL = p.Results.HeightAGL;
                obj.Position = [0, 0, p.Results.HeightAGL];
                obj.TerrainElevation = [];
            end
            
            obj.Frequency = p.Results.Frequency;
            obj.Power = p.Results.Power;
            obj.Gain = p.Results.Gain;
            obj.PatternType = p.Results.PatternType;
            obj.Polarization = p.Results.Polarization;
            obj.Azimuth = p.Results.Azimuth;
            obj.Elevation = p.Results.Elevation;
            obj.BeamWidth = p.Results.BeamWidth;
            obj.Name = p.Results.Name;
        end
        
        function obj = setTerrainPosition(obj, terrain_data)
            % Set the absolute position based on terrain elevation
            % INPUT: terrain_data - TerrainData object
            % OUTPUT: updated antenna config with Position(3) set
            
            if isempty(obj.Location)
                warning('No Location specified for antenna %s', obj.Name);
                return;
            end
            
            % Get terrain elevation at antenna location
            z_terrain = terrain_data.getElevation(obj.Location(1), obj.Location(2));
            
            if isnan(z_terrain)
                error('Antenna %s location [%.1f, %.1f] is outside terrain bounds', ...
                      obj.Name, obj.Location(1), obj.Location(2));
            end
            
            % Set absolute position
            obj.TerrainElevation = z_terrain;
            obj.Position = [obj.Location, z_terrain + obj.HeightAGL];
            
            fprintf('Antenna %s: terrain elevation %.1f m, height AGL %.1f m, total elevation %.1f m MSL\n', ...
                    obj.Name, z_terrain, obj.HeightAGL, obj.Position(3));
        end
        
        function display(obj)
            % Display antenna configuration
            fprintf('\n--- Antenna Configuration: %s ---\n', obj.Name);
            if ~isempty(obj.TerrainElevation)
                fprintf('Location (UTM): [%.2f E, %.2f N]\n', obj.Location);
                fprintf('Terrain elevation: %.2f m MSL\n', obj.TerrainElevation);
                fprintf('Height above ground: %.2f m\n', obj.HeightAGL);
                fprintf('Total elevation: %.2f m MSL\n', obj.Position(3));
            else
                fprintf('Position (UTM): [%.2f, %.2f, %.2f] m\n', obj.Position);
                if ~isempty(obj.HeightAGL)
                    fprintf('Height above ground: %.2f m\n', obj.HeightAGL);
                end
            end
            fprintf('Frequency: %.2f MHz\n', obj.Frequency/1e6);
            fprintf('Power: %.2f W (%.2f dBm)\n', obj.Power, 10*log10(obj.Power*1000));
            fprintf('Gain: %.2f dBi\n', obj.Gain);
            fprintf('Pattern: %s\n', obj.PatternType);
            fprintf('Beamwidth: [%.1f, %.1f] degrees\n', obj.BeamWidth);
        end
    end
end