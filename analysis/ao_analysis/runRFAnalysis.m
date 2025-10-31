function [viewer, tx, rx_ref, power_data] = runRFAnalysis(Config, Z_crop, R_crop, X_grid, Y_grid)
    projCRS = R_crop.ProjectedCRS;

    [Lat_grid, Lon_grid] = projinv(projCRS, X_grid, Y_grid);

    Lat_vec = Lat_grid(:, 1);
    Lon_vec = Lon_grid(1, :);

    viewer = siteviewer(...
        Basemap=none', ...
        Terrain=Z_crop, ...
        TerrainLatitude=Lat_vec, ...
        TerrainLongitude=Lon_vec...
        )
    % setup propogation model
    pm = propagationModel(Config.RF.PropagationModel, Method=Config.RF.Method, UseGPU="auto");
    
    

    % Setup Transmitters
    tx = setupTransmitters(Config, Lat_vec, Lon_vec);
    rx_ref = setupReceivers(Config, Lat_vec, Lon_vec);
    power_data = calculatePower(pm, tx, rx_ref);
end