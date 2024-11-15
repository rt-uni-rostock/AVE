function [lat, lon, alt] = NED2LLA(N, E, D, originLat, originLon, originAlt)
    %ave.NED2LLA Convert local NED frame coordinates to LLA coordinates (WGS84).
    % 
    % PARAMETERS
    % N         ... North value of the NED position (m).
    % E         ... East value of the NED position (m).
    % D         ... Down value of the NED position (m).
    % originLat ... Latitude (rad) that indicates the origin of the local NED frame.
    % originLon ... Longitude (rad) that indicates the origin of the local NED frame.
    % originAlt ... Altitude (m, positive upwards) that indicates the origin of the local NED frame.
    % 
    % RETURN
    % lat ... Latitude (rad) of converted position.
    % lon ... Longitude (rad) of converted position.
    % alt ... Altitude (m, positive upwards) of converted position.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221028    Robert Damerius        Initial release.
    % 20240528    Robert Damerius        Add support for multi-dimensional inputs (N, E, D).
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isequal(size(N),size(E)) && isequal(size(N),size(D)), 'Inputs N, E, D must have the same size!');
    assert(isscalar(originLat), 'Input originLat must be a scalar value!');
    assert(isscalar(originLon), 'Input originLon must be a scalar value!');
    assert(isscalar(originAlt), 'Input originAlt must be a scalar value!');

    % Constants and spheroid properties
    a = 6378137.0;
    oneMinusF = 0.996647189335253;
    e2 = 0.00669437999014132;
    ae2 = 42697.67270718;
    bep2 = 42841.3115133136;

    % ECEF: Computation of (x,y,z) = (x0,y0,z0) + (dx,dy,dz)
    slat = sin(originLat);
    clat = cos(originLat);
    slon = sin(originLon);
    clon = cos(originLon);
    Nval = a / sqrt(1.0 - e2 * slat*slat);
    rho = (Nval + originAlt) * clat;
    x0 = rho * clon;
    y0 = rho * slon;
    z0 = (Nval*(1.0 - e2) + originAlt) * slat;
    t = clat * (-D) - slat * N;
    dz = slat * (-D) + clat * N;
    dx = clon * t - slon * E;
    dy = slon * t + clon * E;
    x = x0 + dx;
    y = y0 + dy;
    z = z0 + dz;
    lon = atan2(y,x);

    % Bowring's formula for initial parametric (beta) and geodetic
    rho = hypot(x,y);
    beta = atan2(z, oneMinusF * rho);
    lat = atan2(z + bep2 * sin(beta).^3, rho - ae2 * cos(beta).^3);

    % Fixed-point iteration with Bowring's formula (typically converges within two or three iterations)
    betaNew = atan2(oneMinusF*sin(lat), cos(lat));
    count = 0;
    while(any(beta(:) ~= betaNew(:)) && count < 5)
        beta = betaNew;
        lat = atan2(z + bep2 * sin(beta).^3, rho - ae2 * cos(beta).^3);
        betaNew = atan2(oneMinusF*sin(lat), cos(lat));
        count = count + 1;
    end

    % Ellipsoidal height from final value for latitude
    slat = sin(lat);
    Nval = a ./ sqrt(1.0 - e2 * slat.^2);
    alt = rho .* cos(lat) + (z + e2 * Nval .* slat) .* slat - Nval;
end

