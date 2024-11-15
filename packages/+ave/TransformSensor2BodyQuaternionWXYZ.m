function quaternionBody = TransformSensor2BodyQuaternionWXYZ(quaternionSensor, R_b2s)
    %ave.TransformSensor2BodyQuaternionWXYZ Transform a unit quaternion measurement from the sensor-frame to the body-frame.
    % 
    % PARAMETERS
    % quaternionSensor ... [double x 4] Input quaternion from sensor, representing orientation from sensor-frame with respect to navigation-frame.
    % R_b2s            ... [double x 3 x 3] Rotation matrix representing the rotation from body-frame to sensor-frame.
    % 
    % RETURN
    % quaternionBody ... [double x 4] Output unit quaternion in body-frame, representing the orientation from the body-frame with respect to the navigation frame.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221101    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert((4 == numel(quaternionSensor)) && isa(quaternionSensor,'double'), 'Input "quaternionSensor" must be a 4-dimensional vector of type double!');
    assert((3 == size(R_b2s,1)) && (3 == size(R_b2s,2)), 'Input "R_b2s" must be a 3-by-3 matrix!');
    q_s2n = [quaternionSensor(1) quaternionSensor(2) quaternionSensor(3) quaternionSensor(4)];
    n = norm(q_s2n);
    if(n < 1e-12)
        q_s2n = [1 0 0 0];
    else
        q_s2n = q_s2n / n;
    end
    q = quatmultiply(q_s2n, rotm2quat(R_b2s));
    quaternionBody = [q(1); q(2); q(3); q(4)];
end

