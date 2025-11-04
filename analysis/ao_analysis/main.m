%% ANTENNA COVERAGE MAPPING - MAIN SCRIPT
% This script demonstrates the complete workflow for antenna coverage analysis
% in the Utah desert using ray tracing propagation
%
% Author: Joe Mama
% Date: 2025
% 
% REQUIREMENTS:
%   - MATLAB R2025a or later
%   - Mapping Toolbox
%   - Signal Processing Toolbox (optional)
%
% WORKFLOW:
%   1. Load terrain data
%   2. Configure antenna(s)
%   3. Setup propagation model
%   4. Compute coverage
%   5. Visualize and export results

clear; close all; clc;

%% ========================================================================
%  STEP 1: LOAD TERRAIN DATA
%  ========================================================================

fprintf('=======================================================\n');
fprintf('  ANTENNA COVERAGE MAPPING - UTAH DESERT\n');
fprintf('=======================================================\n\n');

% Terrain file configuration
filename = 'mdrs_utah_geotif/mdrs.tif';

% Define area of interest (AOI) in UTM coordinates
min_easting_m = 518000.00;   % Min X value (meters)
max_easting_m = 519341.00;   % Max X value (meters)
min_northing_m = 4250734.00; % Min Y value (meters)
max_northing_m = 4251642.00; % Max Y value (meters)

x_limits = [min_easting_m, max_easting_m];
y_limits = [min_northing_m, max_northing_m];

% Load terrain data
terrain = TerrainData(filename, x_limits, y_limits);

resolution_m = 20;
% Visualize terrain (optional)
% terrain.visualize();

%% ========================================================================
%  CONFIGURE ANTENNA(S)
%  ========================================================================

fprintf('\n--- Configuring Antennas ---\n');

antenna1 = AntennaConfig(...
    'Location', [518208.74 , 4250953.52], ...    % [easting, northing] mdrs base station
    'HeightAGL', 3, ...                   
    'Frequency', 915e6, ...                
    'Power', 0.500, ...                        % watts
    'Gain', 12, ...                         % gain dbi
    'PatternType', 'omnidirectional', ...
    'Polarization', 'vertical', ...
    'Name', 'Base_Station_1');

% Example 2: Directional antenna on ridge (not active)
antenna2 = AntennaConfig(...
    'Location', [518800, 4251400], ...    % [easting, northing]
    'HeightAGL', 1, ...                   % 15m mast on ridge
    'Frequency', 915e6, ...
    'Power', 2, ...                        % 2 Watts
    'Gain', 12, ...                        % 12 dBi directional
    'PatternType', 'directional', ...
    'Polarization', 'vertical', ...
    'Azimuth', 180, ...                    % Pointing south
    'Elevation', -5, ...                   % Slight downtilt
    'BeamWidth', [60, 30], ...             % [azimuth, elevation] degrees
    'Name', 'Ridge_Station');

% Store antennas in cell array for batch processing
antennas = {antenna1};

%% ========================================================================
%  SETUP PROPAGATION MODEL
%  ========================================================================

fprintf('\n--- Configuring Propagation Model ---\n');

% Create ray tracing propagation model
prop_model = RayTracingPropagation(terrain, ...
    'MaxRange', 1500, ...              % Maximum range in meters
    'RayResolution', resolution_m, ... % Ray tracing resolution
    'NumReflections', 2, ...           % Consider up to 2 reflections
    'GroundPermittivity', 4, ...       % Utah desert relative permittivity
    'GroundConductivity', 0.005, ...   % Desert conductivity (S/m)
    'AtmosphericLoss', 0.1);           % Atmospheric loss (dB/km)

fprintf('Propagation model configured for NLOS desert environment\n');

%% ========================================================================
%  SETUP COVERAGE MAPPER
%  ========================================================================

fprintf('\n--- Configuring Coverage Mapper ---\n');

% Create coverage mapper
mapper = CoverageMapper(terrain, prop_model, ...
    'ReceiverHeight', 1.5, ...         % Receiver height above ground (m)
    'ReceiverSensitivity', -72, ...   % Minimum detectable signal (dBm)
    'GridSpacing', resolution_m);                % Coverage grid spacing (m)

fprintf('Coverage mapper configured\n');

%% ========================================================================
%  COMPUTE COVERAGE FOR EACH ANTENNA
%  ========================================================================

coverage_maps = cell(length(antennas), 1);

for i = 1:length(antennas)
    coverage_maps{i} = mapper.computeCoverage(antennas{i});
end

%% ========================================================================
%  COMBINE MULTIPLE ANTENNA COVERAGE
%  ========================================================================

if length(antennas) > 1
    fprintf('\n--- Combining Coverage Maps ---\n');
    combined_coverage = mapper.combineCoverage(coverage_maps);
end

%% ========================================================================
%  VISUALIZE RESULTS
%  ========================================================================

fprintf('\n--- Generating Visualizations ---\n');

% Visualization options
viz_options = struct();
viz_options.ShowTerrain = true;
viz_options.Show3D = false;
viz_options.ShowLOS = false;
viz_options.ColorLimits = [-100, -40];  % dBm range

% Visualize individual antenna coverage
for i = 1:length(antennas)
    mapper.visualizeCoverage(coverage_maps{i}, viz_options);
    
    viz_options.SavePath = sprintf('coverage_%s.png', antennas{i}.Name);
    mapper.visualizeCoverage(coverage_maps{i}, viz_options);
end

% Visualize combined coverage
if length(antennas) > 1
    figure('Position', [100, 100, 1200, 900]);
    mapper.visualizeCoverage(combined_coverage, viz_options);
    title('Combined Multi-Antenna Coverage Map');
end

% LOS visualization
viz_options.ShowLOS = false;
figure('Position', [100, 100, 1200, 900]);
mapper.visualizeCoverage(coverage_maps{1}, viz_options);
title(sprintf('Line-of-Sight Analysis: %s', antennas{1}.Name));

%% ========================================================================
%  EXPORT RESULTS
%  ========================================================================

fprintf('\n--- Exporting Results ---\n');

% Export to GeoTIFF
% mapper.exportToGeoTIFF(coverage_maps{1}, 'coverage_antenna1.tif');

% Save coverage data to MAT file
% save('coverage_results.mat', 'coverage_maps', 'combined_coverage', ...
%      'antennas', 'terrain');
