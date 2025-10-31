% --- ConfigurationFile.m (The master file for all parameters) ---
function Config = loadProjectConfig()

Config.FileName = 'mdrs_utah_geotif/merged.tif';

% Your desired crop coordinates (UTM meters, Zone 12N)
Config.Crop.EastingLimits = [518084.98, 519163.59];
Config.Crop.NorthingLimits = [4250745.22, 4251555.30];
Config.Crop.UTM_Zone = '12S';

% Data Clamping/Cleaning
Config.Data.LowerPercentile = 1; % Clamps data below this percentile
Config.Data.UpperPercentile = 99; % Clamps data above this percentile


% --- RF AND SITE PARAMETERS ---
Config.RF.PropagationModel = "raytracing";
Config.RF.Method = 'sbr';
Config.RF.Frequency = 2.4e9; % Operating frequency (Hz)
Config.RF.Tx_Power = 30; % Transmitter power (dBm)
Config.RF.Rx_Sensitivity = -90; % Minimum signal required (dBm)

% Transmitter (TX) Base Station
Config.Tx.Easting = 491000;
Config.Tx.Northing = 4800500;
Config.Tx.AntennaHeight_AGL = 1.5; % Meters


% Rover Target Endpoints (RX Markers)
Config.Rx.Targets = [...
    490500, 4800200; % [Easting, Northing] Pair 1
    491500, 4800800; % [Easting, Northing] Pair 2
    491200, 4800300  % [Easting, Northing] Pair 3
];
Config.Rx.AntennaHeight = 0.9; % Rover antenna height (m)


% --- C. PLOTTING/VISUALIZATION SETTINGS ---
Config.Plot.SignalStrengths = [-100, -90, -80, -70, -60]; % Coverage contours
Config.Plot.MarkerStyle = '^';
Config.Plot.MarkerColor = 'r';

end