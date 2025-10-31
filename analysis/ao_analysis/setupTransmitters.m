function [tx_site] = SetupTransmitter(Config)
    projCRS = R_crop.ProjectedCRS;

    [Lat_grid, Lon_grid] = projinv(projCRS, X_grid, Y_grid);

    Lat_vec = Lat_grid(:, 1);
    Lon_grid = Lon_grid(1, :);
    
    % TODO: Using meaningful Names and generate from 
    % predefined transmitter arrays in Config
    tx_site = txsite(Name="Transmitter", Latitude=Lat_vec, Longitude=Lon_grids);
end