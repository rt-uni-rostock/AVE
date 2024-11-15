function [outPositionLLA, outOrientationQuaternionWXYZ, outVelocityUVW] = StrapdownAlgorithm(sampletime, imuAcceleration, imuAngularRate, positionLLA, orientationQuaternionWXYZ, velocityUVW)
    %ave.StrapdownAlgorithm Calculate one discrete integration step of the strapdown algorithm. The explicit euler method is used as integration method.
    % The strapdown algorithm predicts the motion on earth for given inertial measurements (acceleration and angular rate).
    % 
    % PARAMETERS
    % sampletime                ... [double x 1] Sampletime to be used for integration. Unit is (s).
    % imuAcceleration           ... [double x 3] Raw acceleration of the IMU. The acceleration bias may be removed. Unit is (m/s^2).
    % imuAngularRate            ... [double x 3] Raw angular rate of the IMU. The angular rate bias may be removed. Unit is (rad/s).
    % positionLLA               ... [double x 3] Initial geographic position (latitude [rad], longitude [rad], altitude [m]). The altitude is counted positive upwards.
    % orientationQuaternionWXYZ ... [double x 4] Initial unit quaternion representing the orientation of the b-frame with respect to the n-frame.
    %                               The format is [w;x;y;z], where w denotes the scalar part and [x;y;z] denotes the vector part of the quaternion, respectively.
    % velocityUVW               ... [double x 3] Initial body-fixed velocity. Unit is (m/s).
    % 
    % 
    % RETURN
    % outPositionLLA               ... [double x 3] Output geographic position. The format is equal to the positionLLA input.
    % outOrientationQuaternionWXYZ ... [double x 4] Output quaternion. The format is equal to the orientationQuaternionWXYZ input.
    % outVelocityUVW               ... [double x 3] Output velocity. The format is equal to the velocityUVW input.
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    % Ensure correct inputs
    assert(isscalar(sampletime) && isa(sampletime,'double'), 'Input "sampletime" must be a scalar of type double!');
    assert((3 == numel(imuAcceleration)) && isa(imuAcceleration,'double'), 'Input "imuAcceleration" must be a 3-dimensional vector of type double!');
    assert((3 == numel(imuAngularRate)) && isa(imuAngularRate,'double'), 'Input "imuAngularRate" must be a 3-dimensional vector of type double!');
    assert((3 == numel(positionLLA)) && isa(positionLLA,'double'), 'Input "positionLLA" must be a 3-dimensional vector of type double!');
    assert((4 == numel(orientationQuaternionWXYZ)) && isa(orientationQuaternionWXYZ,'double'), 'Input "orientationQuaternionWXYZ" must be a 4-dimensional vector of type double!');
    assert((3 == numel(velocityUVW)) && isa(velocityUVW,'double'), 'Input "velocityUVW" must be a 3-dimensional vector of type double!');
    imuAcceleration = reshape(imuAcceleration, [3 1]);
    imuAngularRate = reshape(imuAngularRate, [3 1]);
    positionLLA = reshape(positionLLA, [3 1]);
    orientationQuaternionWXYZ = reshape(orientationQuaternionWXYZ, [4 1]);
    velocityUVW = reshape(velocityUVW, [3 1]);
    qn = norm(orientationQuaternionWXYZ);
    if(qn < 1e-12)
        orientationQuaternionWXYZ = [1;0;0;0];
    else
        orientationQuaternionWXYZ = orientationQuaternionWXYZ / qn;
    end

    % WGS84 information for initial position and rotation matrices for initial orientation
    a = 6378137.0;
    ee = 0.00669437999014132;
    s = sin(positionLLA(1));
    invRoot = 1.0 ./ sqrt(1 - ee * s .* s);
    Re = a .* invRoot;
    Rn = Re .* (1-ee) .* (invRoot.*invRoot);
    omegaEarth = 7.292115e-5;
    localGravity = ave.GravityWGS84(positionLLA(1), positionLLA(2), positionLLA(3));
    R_b2n = quat2rotm(orientationQuaternionWXYZ');
    R_n2b = R_b2n';

    % Transform velocity from b-frame to n-frame
    velocityNED = R_b2n * velocityUVW;

    % Predict position and ensure correct format
    outPositionLLA = positionLLA + sampletime * diag([1/(Rn + positionLLA(3)), 1/((Re + positionLLA(3)) * cos(positionLLA(1))), -1]) * velocityNED;
    [lat, lon] = EnsureLatLonRange(outPositionLLA(1), outPositionLLA(2));
    outPositionLLA(1) = lat;
    outPositionLLA(2) = lon;

    % Predict velocity
    w_ie = [omegaEarth * cos(positionLLA(1)); 0; -omegaEarth * sin(positionLLA(1))];
    w_en = [(velocityNED(2) / ((Re + positionLLA(3)))); -(velocityNED(1) / (Rn + positionLLA(3))); -(velocityNED(2) / (Re + positionLLA(3)))*tan(positionLLA(1))];
    coriolis = cross(2 * w_ie + w_en, velocityNED);
    outVelocityUVW = R_n2b * (velocityNED + sampletime * (R_b2n * imuAcceleration - coriolis + [0.0; 0.0; localGravity]));

    % Predict attitude and ensure normalized quaternion
    w_nb = imuAngularRate - R_n2b * (w_ie + w_en);
    Omega = [0 -w_nb(1) -w_nb(2) -w_nb(3); w_nb(1) 0 w_nb(3) -w_nb(2); w_nb(2) -w_nb(3) 0 w_nb(1); w_nb(3) w_nb(2) -w_nb(1) 0];
    outOrientationQuaternionWXYZ = (eye(4)*(1 - sampletime*sampletime*(w_nb(1)*w_nb(1) + w_nb(2)*w_nb(2) + w_nb(3)*w_nb(3))/8) + 0.5*sampletime*Omega) * orientationQuaternionWXYZ;
    qn = norm(outOrientationQuaternionWXYZ);
    if(qn < 1e-12)
        outOrientationQuaternionWXYZ = [1;0;0;0];
    else
        outOrientationQuaternionWXYZ = outOrientationQuaternionWXYZ / qn;
    end
end

function [lat, lon] = EnsureLatLonRange(phi, lambda)
    %EnsureLatLonRange Convert phi [rad] and lambda [rad] to latitude [rad] and longitude [rad] with latitude being in range [-pi/2, +pi/2] and longitude being in range [-pi; +pi).
    % 
    % PARAMETERS
    % phi    ... Input angle for latitude in radians.
    % lambda ... Input angle for longitude in radians.
    % 
    % RETURN
    % lat ... Output angle for latitude [rad] being in range [-pi/2, +pi/2].
    % lon ... Output angle for longitude [rad] being in range [-pi, +pi).
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    const_pi_half = 0.5 * pi;
    const_pi_2 = (pi + pi);

    % Make sure that inputs are scalars
    assert(isscalar(phi), 'Input "phi" must be scalar!');
    assert(isscalar(lambda), 'Input "lambda" must be scalar!');

    % Convert phi to symmetric angle [-pi, pi)
    phi = double(mod(phi, const_pi_2));
    phi = phi - double(phi >= pi) * const_pi_2;

    % Check for (phi > pi/2) and (phi < -pi/2)
    if(phi > const_pi_half)
        phi = pi - phi;
        lambda = lambda + pi;
    elseif(phi < -const_pi_half)
        phi = -pi - phi;
        lambda = lambda + pi;
    end

    % Convert lambda to symmetric angle [-pi, pi)
    lambda = double(mod(lambda, const_pi_2));
    lambda = lambda - double(lambda >= pi) * const_pi_2;

    % Set output
    lat = phi;
    lon = lambda;
end

