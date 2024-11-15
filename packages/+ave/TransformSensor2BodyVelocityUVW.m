function [uBody, vBody, wBody] = TransformSensor2BodyVelocityUVW(uSensor, vSensor, wSensor, R_b2s, r_b2s, pqr)
    %ave.TransformSensor2BodyVelocityUVW Transform a velocity measurement from the sensor-frame to the body frame.
    % 
    % PARAMETERS
    % uSensor   ... [double x 1] U velocity measurement of the sensor in m/s w.r.t. the navigation frame, given in the sensor frame.
    % vSensor   ... [double x 1] V velocity measurement of the sensor in m/s w.r.t. the navigation frame, given in the sensor frame.
    % wSensor   ... [double x 1] W velocity measurement of the sensor in m/s w.r.t. the navigation frame, given in the sensor frame.
    % R_b2s     ... [double x 3 x 3] Rotation matrix representing the rotation from body-frame to sensor-frame.
    % r_b2s     ... [double x 3] Vector that indicates the sensor position w.r.t. the body frame in body frame coordinates (meters).
    % pqr       ... [double x 3] Vector of angular velocities in rad/s (angular rate of body-frame w.r.t. navigation-frame in b-frame coordinates).
    % 
    % RETURN
    % uBody ... [double x 1] U velocity of the body-frame origin in m/s.
    % vBody ... [double x 1] V velocity of the body-frame origin in m/s.
    % wBody ... [double x 1] W velocity of the body-frame origin in m/s.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221101    Robert Damerius        Initial release.
    % 20230824    Robert Damerius        Change input order of R_b2s and r_b2s for consistency with other transform functions.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isscalar(uSensor) && isscalar(vSensor) && isscalar(wSensor), 'Inputs "uSensor", "vSensor", "wSensor" must be scalars!');
    assert((3 == size(R_b2s,1)) && (3 == size(R_b2s,2)), 'Input "R_b2s" must be a 3-by-3 matrix!');
    r_b2s = reshape(r_b2s, [3 1]);
    pqr = reshape(pqr, [3 1]);
    uvw = R_b2s' * [uSensor; vSensor; wSensor] - cross(pqr, r_b2s);
    uBody = uvw(1);
    vBody = uvw(2);
    wBody = uvw(3);
end

