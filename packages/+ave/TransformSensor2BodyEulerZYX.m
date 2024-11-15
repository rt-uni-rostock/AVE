function [rollBody, pitchBody, yawBody] = TransformSensor2BodyEulerZYX(rollSensor, pitchSensor, yawSensor, R_b2s)
    %ave.TransformSensor2BodyEulerZYX Transform an orientation measurement given as euler angles (ZYX-convention) from the sensor-frame to the body frame.
    % 
    % PARAMETERS
    % rollSensor  ... [double x 1] Roll angle measurement of the sensor in rad w.r.t. the navigation frame, given in the sensor frame.
    % pitchSensor ... [double x 1] Pitch angle measurement of the sensor in rad w.r.t. the navigation frame, given in the sensor frame.
    % yawSensor   ... [double x 1] Yaw angle measurement of the sensor in rad w.r.t. the navigation frame, given in the sensor frame.
    % R_b2s       ... [double x 3 x 3] Rotation matrix representing the rotation from body-frame to sensor-frame.
    % 
    % RETURN
    % rollBody  ... [double x 1] Roll angle of the body-frame in rad.
    % pitchBody ... [double x 1] Pitch angle of the body-frame in rad.
    % yawBody   ... [double x 1] Yaw angle of the body-frame in rad.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230824    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isscalar(rollSensor) && isscalar(pitchSensor) && isscalar(yawSensor), 'Inputs "rollSensor", "pitchSensor", "yawSensor" must be scalars!');
    assert((3 == size(R_b2s,1)) && (3 == size(R_b2s,2)), 'Input "R_b2s" must be a 3-by-3 matrix!');
    q = eul2quat([yawSensor, pitchSensor, rollSensor]);
    q = ave.TransformSensor2BodyQuaternionWXYZ([q(1); q(2); q(3); q(4)], R_b2s);
    eul = quat2eul([q(1), q(2), q(3), q(4)]);
    rollBody = ave.SymmetricalAngle(eul(3));
    pitchBody = ave.SymmetricalAngle(eul(2));
    yawBody = ave.SymmetricalAngle(eul(1));
end

