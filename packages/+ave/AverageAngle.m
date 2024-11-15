function avgAngle = AverageAngle(angle1, angle2)
    %ave.AverageAngle Calculate the average angle from two given angles in radians.
    % 
    % PARAMETERS
    % angle1 ... [double] Scalar input angle 1 in radians.
    % angle2 ... [double] Scalar input angle 2 in radians.
    % 
    % RETURN
    % avgAngle ... [double] Average angle in radians being in range [-pi, pi).
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20201028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    avgAngle = ave.SymmetricalAngle(angle1 + 0.5 * ave.SymmetricalAngle(angle2 - angle1));
end

