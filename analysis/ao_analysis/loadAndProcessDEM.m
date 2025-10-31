function [Z_crop, R_crop, X_grid, Y_grid] = loadAndProcessDEM(Config)
    [Z_full, R_full] = readgeoraster(Config.filename, OutputType="double");
    [Z_crop, R_crop] = mapcrop(Z_full, R_full, Config.xlimits, Config.y_limits);
    

    x_vec = linspace(R_crop.XWorldLimits(1), R_crop.XWorldLimits(2), R_crop.RasterSize(2));
    y_vec = linspace(R_crop.YWorldLimits(1), R_crop.YWorldLimits(2), R_crop.RasterSize(1));
    [X_grid, Y_grid] = meshgrid(x_vec, y_vec);

end