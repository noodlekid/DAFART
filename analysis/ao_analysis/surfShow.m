map = "mdrs_utah_geotif/merged.tif";
filename = map;

min_easting_m = 518084.98;  % Min X value (meters)
max_easting_m = 519163.59;  % Max X value (meters)
min_northing_m = 4250745.22; % Min Y value (meters)
max_northing_m = 4251555.30; % Max Y value (meters)


x_limits = [min_easting_m, max_easting_m];
y_limits = [min_northing_m, max_northing_m];


[Z_full, R_full] = readgeoraster(filename);
Z_full = double(Z_full);

disp('Analyzing data for missing value (NoData)...');

nodata_val = min(Z_full(:));
valid_data_indices = (Z_full > nodata_val);

if sum(valid_data_indices(:)) == 0
    error('File contains NO valid data (all pixels are the same minimum value).');
end

actual_mean_elev = mean(Z_full(valid_data_indices), 'omitnan');

if nodata_val < (actual_mean_elev - 1000)
    Z_full(Z_full == nodata_val) = NaN;
else
    disp('No suspicious minimum value found. Proceeding without NaN mask.');
end



[Z_crop, R_crop] = mapcrop(Z_full, R_full, x_limits, y_limits);

if isempty(Z_crop)
    error('Crop resulted in an empty matrix. Your limits may be outside the file extent.');
end

valid_data_in_crop = Z_crop(~isnan(Z_crop));

if isempty(valid_data_in_crop)
    error(['Crop failed: Selected area contains NO valid data. ', ...
           'Please verify your UTM coordinates are within the file''s footprint (check the UTM Zone!).']);
end

disp(['Crop successful. Valid elevation range: ', ...
      num2str(min(valid_data_in_crop)), ' / ', num2str(max(valid_data_in_crop))]);


x_vec = linspace(R_crop.XWorldLimits(1), R_crop.XWorldLimits(2), R_crop.RasterSize(2));
y_vec = linspace(R_crop.YWorldLimits(1), R_crop.YWorldLimits(2), R_crop.RasterSize(1));
[X_grid, Y_grid] = meshgrid(x_vec, y_vec);

figure;
h_surf = surf(X_grid, Y_grid, Z_crop);

shading interp;
camlight right;
lighting phong;

proj_name = R_full.ProjectedCRS.Name;

xlabel('UTM Easting (m)');
ylabel('UTM Northing (m)');
zlabel('Elevation (m)');
title({'3D Terrain Map (Cropped Section)', ['Projection: ', proj_name]});
colorbar;

axis equal;
axis 'auto z'; 