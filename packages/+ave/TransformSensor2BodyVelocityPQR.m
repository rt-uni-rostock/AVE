function [pBody, qBody, rBody] = TransformSensor2BodyVelocityPQR(pSensor, qSensor, rSensor, R_b2s)
    %ave.TransformSensor2BodyVelocityPQR Transform an angular velocity measurement from the sensor-frame to the body frame.
    % 
    % PARAMETERS
    % pSensor   ... [double x 1] P velocity measurement of the sensor in rad/s w.r.t. the navigation frame, given in the sensor frame.
    % qSensor   ... [double x 1] Q velocity measurement of the sensor in rad/s w.r.t. the navigation frame, given in the sensor frame.
    % rSensor   ... [double x 1] R velocity measurement of the sensor in rad/s w.r.t. the navigation frame, given in the sensor frame.
    % R_b2s     ... [double x 3 x 3] Rotation matrix representing the rotation from body-frame to sensor-frame.
    % 
    % RETURN
    % pBody ... [double x 1] P velocity of the body-frame origin in rad/s.
    % qBody ... [double x 1] Q velocity of the body-frame origin in rad/s.
    % rBody ... [double x 1] R velocity of the body-frame origin in rad/s.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230824    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isscalar(pSensor) && isscalar(qSensor) && isscalar(rSensor), 'Inputs "pSensor", "qSensor", "rSensor" must be scalars!');
    assert((3 == size(R_b2s,1)) && (3 == size(R_b2s,2)), 'Input "R_b2s" must be a 3-by-3 matrix!');
    pqr = R_b2s' * [pSensor; qSensor; rSensor];
    pBody = pqr(1);
    qBody = pqr(2);
    rBody = pqr(3);
end

