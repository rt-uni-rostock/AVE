function varargout = LLA2NED(varargin)
    %ave.LLA2NED Convert LLA coordinates (WGS84) to local NED frame coordinates.
    % This implementation uses the algorithm proposed by
    %     S. P. Drake
    %     Converting GPS Coordinates to Navigation Coordinates
    %     Surveillance Systems Division, Electronics and Surveillance Research Laboratory, DSTO-TN-0432
    %     April, 2002
    % 
    % This function can be called in two different ways, either scalar-like or column-vector-like:
    %     [N, E, D] = ave.LLA2NED(lat, lon, alt, originLat, originLon, originAlt);
    %     NED = ave.LLA2NED(LLA, originLLA);
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
    assert(2 == nargin || 6 == nargin);
    if(2 == nargin)
        lat = varargin{1}(1,:);
        lon = varargin{1}(2,:);
        alt = varargin{1}(3,:);
        originLat = varargin{2}(1,:);
        originLon = varargin{2}(2,:);
        originAlt = varargin{2}(3,:);
    elseif(6 == nargin)
        lat = varargin{1};
        lon = varargin{2};
        alt = varargin{3};
        originLat = varargin{4};
        originLon = varargin{5};
        originAlt = varargin{6};
    end
    [N, E, D] = LLA2NED_implementation(lat, lon, alt, originLat, originLon, originAlt);
    if(2 == nargin)
        varargout{1} = [N; E; D];
    elseif(6 == nargin)
        varargout{1} = N;
        varargout{2} = E;
        varargout{3} = D;
    end
end

function [N, E, D] = LLA2NED_implementation(lat, lon, alt, originLat, originLon, originAlt)
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

