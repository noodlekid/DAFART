classdef TerrainData
    % TERRAINDATA Class for handling GeoTIFF terrain data
    % Manages terrain elevation data and coordinate transformations
    
    properties
        Z               % Elevation matrix (meters)
        R               % Spatial referencing object
        X               % Easting grid (meters, UTM)
        Y               % Northing grid (meters, UTM)
        Resolution      % Grid resolution in meters
        Limits          % [min_x, max_x, min_y, max_y]
    end
    
    methods
        function obj = TerrainData(filename, x_limits, y_limits)
            % Constructor - loads and crops terrain data
            % INPUTS:
            %   filename: path to GeoTIFF file
            %   x_limits: [min_easting, max_easting] in meters
            %   y_limits: [min_northing, max_northing] in meters
            
            fprintf('Loading terrain data from: %s\n', filename);
            
            % Read full GeoTIFF
            [Z_full, R_full] = readgeoraster(filename);
            Z_full = double(Z_full);
            
            % Crop to area of interest
            [obj.Z, obj.R] = mapcrop(Z_full, R_full, x_limits, y_limits);
            
            % Generate coordinate grids
            [rows, cols] = size(obj.Z);
            obj.Resolution = obj.R.CellExtentInWorldX;
            
            % Create X and Y grids
            x_vec = obj.R.XWorldLimits(1) + obj.Resolution/2 : obj.Resolution : obj.R.XWorldLimits(2) - obj.Resolution/2;
            y_vec = obj.R.YWorldLimits(2) - obj.Resolution/2 : -obj.Resolution : obj.R.YWorldLimits(1) + obj.Resolution/2;
            
            [obj.X, obj.Y] = meshgrid(x_vec, y_vec);
            
            % Store limits
            obj.Limits = [min(obj.X(:)), max(obj.X(:)), min(obj.Y(:)), max(obj.Y(:))];
            
            fprintf('Terrain loaded: %d x %d grid, %.2f m resolution\n', ...
                    rows, cols, obj.Resolution);
            fprintf('Elevation range: %.2f to %.2f m\n', min(obj.Z(:)), max(obj.Z(:)));
        end
        
        function z = getElevation(obj, x, y)
            % Get elevation at specific coordinates using interpolation
            % INPUTS:
            %   x, y: coordinates in UTM meters (can be arrays)
            % OUTPUT:
            %   z: elevation in meters
            
            z = interp2(obj.X, obj.Y, obj.Z, x, y, 'linear', nan);
        end
        
        function visualize(obj, fig_handle)
            % Visualize terrain in 3D
            if nargin < 2
                fig_handle = figure;
            else
                figure(fig_handle);
            end
            
            surf(obj.X, obj.Y, obj.Z, 'EdgeColor', 'none', 'FaceColor', 'interp');
            colormap(terrain);
            colorbar;
            xlabel('Easting (m)');
            ylabel('Northing (m)');
            zlabel('Elevation (m)');
            title('Terrain Elevation Map');
            axis tight;
            view(45, 30);
            lighting gouraud;
            camlight;
        end
    end
end