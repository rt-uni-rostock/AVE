function [N, E, D] = LLA2NED(lat, lon, alt, originLat, originLon, originAlt)
    %ave.LLA2NED Convert LLA coordinates (WGS84) to local NED frame coordinates.
    % This implementation uses the algorithm proposed by
    %     S. P. Drake
    %     Converting GPS Coordinates to Navigation Coordinates
    %     Surveillance Systems Division, Electronics and Surveillance Research Laboratory, DSTO-TN-0432
    %     April, 2002
    % 
    % PARAMETERS
    % lat ... Latitude (rad) of position that should be converted to NED.
    % lon ... Longitude (rad) of position that should be converted to NED.
    % alt ... Altitude (m, positive upwards) of position that should be converted to NED.
    % originLat ... Latitude (rad) that indicates the origin of the local NED frame.
    % originLon ... Longitude (rad) that indicates the origin of the local NED frame.
    % originAlt ... Altitude (m, positive upwards) that indicates the origin of the local NED frame.
    % 
    % RETURN
    % N ... North value of the NED position (m).
    % E ... East value of the NED position (m).
    % D ... Down value of the NED position (m).
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20201028    Robert Damerius        Initial release.
    % 20240528    Robert Damerius        Add support for multi-dimensional inputs (lat, lon, alt).
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isequal(size(lat),size(lon)) && isequal(size(lat),size(alt)), 'Inputs lat, lon, alt must have the same size!');
    assert(isscalar(originLat), 'Input originLat must be a scalar value!');
    assert(isscalar(originLon), 'Input originLon must be a scalar value!');
    assert(isscalar(originAlt), 'Input originAlt must be a scalar value!');

    % Constants according to WGS84
    a = 6378137.0;
    e2 = 0.00669437999014132;

    % Location of data points
    dphi = lat - originLat;
    dlam = ave.SymmetricalAngle(lon - originLon);
    dh = alt - originAlt;

    % Some useful definitions
    cp = cos(originLat);
    sp = sin(originLat);
    tmp1 = sqrt(1-e2*sp*sp);
    tmp3 = tmp1*tmp1*tmp1;
    dlam2 = dlam.*dlam;
    dphi2 = dphi.*dphi;

    % Transformations
    E = (a/tmp1+originAlt)*cp*dlam - (a*(1-e2)/tmp3+originAlt)*sp*dphi.*dlam +cp*dlam.*dh;
    N = (a*(1-e2)/tmp3 + originAlt)*dphi + 1.5*cp*sp*a*e2*dphi2 + sp*sp*dh.*dphi + 0.5*sp*cp*(a/tmp1 +originAlt)*dlam2;
    D = -(dh - 0.5*(a-1.5*a*e2*cp*cp+0.5*a*e2+originAlt)*dphi2 - 0.5*cp*cp*(a/tmp1 -originAlt)*dlam2);
end

