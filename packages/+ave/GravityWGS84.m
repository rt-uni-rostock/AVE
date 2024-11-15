function gravity = GravityWGS84(latitude, longitude, altitude)
    %ave.GravityWGS84 Calculate the local gravity [m/s^2] for a given position according to the WGS84 reference model. This implementation
    % is based on MATLAB's gravitywgs84 function using the close approximation method, e.g. gravitywgs84(h,lat,lon,'CloseApprox',[false false false 0])
    % including atmosphere and centrifugal effects and excluding the presence of a precessing reference frame.
    % 
    % PARAMETERS
    % latitude   ... [double] Latitude in radians.
    % longitude  ... [double] Longitude in radians.
    % altitude   ... [double] Altitude in meters, positive upwards.
    % 
    % RETURN
    % gravity    ... [double] Local gravity in m/s^2
    % 
    % DETAILS
    % Use of the WGS84 Close Approximation model should be limited to a geodetic height of 20,000.0 meters (approximately 65,620.0 feet).
    % Below this height, it gives results with submicrogal precision (see help of gravitywgs84).
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert((numel(latitude) == numel(longitude)) && (numel(latitude) == numel(altitude)), 'Inputs must have the same number of elements!');
    gravity = latitude;
    for i = 1:numel(latitude)
        gravity(i) = GravityWGS84Scalar(latitude(i), longitude(i), altitude(i));
    end
end

function gravity = GravityWGS84Scalar(latitude, longitude, altitude)
    % Some constants
    E        = 5.2185400842339e+5;  % = WGS->E;
    E2       = E*E;                 % = WGS84->E * WGS84->E;
    GM       = 3986004.418e+8;      % = WGS84->GM_default;
    a        = 6378137.0;           % = WGS->a;
    b        = 6356752.3142;        % = WGS->b;
    e2       = 6.69437999014e-3;    % = WGS->e2;
    b_over_a = 0.996647189335;      % = WGS->b_over_a;
    omega    = 7.292115e-5;         % = WGS->omega_default;

    % Precalculation of sin/cos
    sinphi = sin(latitude);
    cosphi = cos(latitude);
    sinlambda = sin(longitude);
    coslambda = cos(longitude);
    sin2phi = sinphi * sinphi;

    %/* Radius of Curvature in prime vertical (N) /eq. 4-15/ */
    N = (a) / (sqrt(1.0 - (e2) * sin2phi));

    %/* Calculate rectangular coordinates /eq. 4-14/ */
    x_rec = (N + altitude) * cosphi * coslambda;
    y_rec = (N + altitude) * cosphi * sinlambda;
    z_rec = ((b_over_a) * (b_over_a) * N + altitude) * sinphi;

    %/* Calculate various parameters */
    D = x_rec * x_rec + y_rec * y_rec + z_rec * z_rec - E2;
    u2 = 0.5 * D * (1.0 + sqrt(1.0 + 4.0 * E2 * z_rec * z_rec / (D * D)));
    u2E2 = u2 + E2;

    %/* /eq. 4-8/ */
    u = sqrt(u2);

    %/* /eq. 4-9/ */
    beta = atan(z_rec * sqrt(u2E2) / (u * sqrt(x_rec * x_rec + y_rec * y_rec)));

    %/* generate common sines and cosines */
    sinbeta = sin(beta);
    cosbeta = cos(beta);
    sin2beta = sinbeta * sinbeta;
    cos2beta = cosbeta * cosbeta;

    %/* /eq. 4-10/ */
    w = sqrt((u2 + E2 * sin2beta) / (u2E2));

    %/* /eq. 4-11/ */
    q = 0.5 * ((1.0 + 3.0 * u2 / (E2)) * atan((E) / u) - 3.0 * u / (E));

    %/* /eq. 4-12/ */
    qo = 0.5 * ((1.0 + 3.0 * (b) * (b) / (E2)) * atan((E) / (b)) - 3.0 * (b) / (E));

    %/* /eq. 4-13/ */
    q_prime = 3.0 * ((1.0 + u2 / (E2)) * (1.0 - (u / (E)) * atan((E) / u))) - 1.0;

    %/* Use Centrifugal Force */
    cf_u = u * cos2beta * omega * omega / w;
    cf_beta = sqrt(u2E2) * cosbeta * sinbeta * omega * omega / w;

    %/* /eq. 4-5/ */
    gamma_u = -(GM / u2E2 + omega * omega * (a) * (a) * (E) * q_prime * (0.5 * sin2beta - 1.0 / 6.0) / (u2E2 * qo)) / w + cf_u;

    %/* /eq. 4-6/ */
    gamma_beta = omega * omega * (a) * (a) * q * sinbeta * cosbeta / (sqrt(u2E2) * w * qo) - cf_beta;

    % Resulting gravity
    gravity = sqrt(gamma_u*gamma_u + gamma_beta*gamma_beta);
end

