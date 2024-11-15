function y = SymmetricalAngle(x)
    %ave.SymmetricalAngle Convert a given angle x [rad] to an output angle y [rad] with y being in range [-pi, +pi).
    % 
    % PARAMETERS
    % x ... Input angle in radians, either scalar or n-dimensional.
    % 
    % RETURN
    % y ... Output angle in radians being in range [-pi, pi).
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20201028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    pi2 = pi + pi;
    x = x - pi2 * fix(x / pi2);
    y = x + pi2 * (double(x < -pi) - double(x >= pi));
end

