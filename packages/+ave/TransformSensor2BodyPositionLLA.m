function [latBody, lonBody, altBody] = TransformSensor2BodyPositionLLA(latSensor, lonSensor, altSensor, R_b2n, r_b2s)
    %ave.TransformSensor2BodyPositionLLA Transform a geographic position (WGS84) from the sensor frame to the body-frame.
    % 
    % PARAMETERS
    % latSensor ... [double x 1] Latitude measurement of the sensor in radians, given in the sensor frame.
    % lonSensor ... [double x 1] Longitude measurement of the sensor in radians, given in the sensor frame.
    % altSensor ... [double x 1] Altitude measurement of the sensor in meters (positive upwards), given in the sensor frame.
    % R_b2n     ... [double x 3 x 3] Rotation matrix representing the rotation from body-frame to navigation frame.
    % r_b2s     ... [double x 3] Vector that indicates the sensor position w.r.t. the body frame in body frame coordinates (meters).
    % 
    % RETURN
    % latBody ... [double x 1] Latitude of the body-frame origin in radians.
    % lonBody ... [double x 1] Longitude of the body-frame origin in radians.
    % altBody ... [double x 1] Altitude of the body-frame origin in meters (positive upwards).
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221101    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isscalar(latSensor) && isscalar(lonSensor) && isscalar(altSensor), 'Inputs "latSensor", "lonSensor", "altSensor" must be scalars!');
    assert((3 == size(R_b2n,1)) && (3 == size(R_b2n,2)), 'Input "R_b2n" must be a 3-by-3 matrix!');
    r_NED = -R_b2n * reshape(r_b2s, [3 1]);
    [latBody, lonBody, altBody] = ave.NED2LLA(r_NED(1),r_NED(2),r_NED(3),latSensor,lonSensor,altSensor);
end

