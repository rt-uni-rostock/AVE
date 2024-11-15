function y = Smoothstep(edge0, edge1, x)
    %ave.Smoothstep A smooth interpolation function between two values [edge0, edge1] with f(edge0)=0, f(edge1)=1 and df(edge0)/dx = df(edge1)/dx = 0.
    % The function is defined to be the hermite interpolation after clamping.
    %                 { 0                if x <= 0
    % Smoothstep(x) = { 3*x^2 - 2*x^3    if 0 <= x <= 1
    %                 { 1                if 1 <= x
    % 
    % PARAMETERS
    % edge0 ... Edge value where the output should be zero.
    % edge1 ... Edge value where the output should be one.
    % x     ... Function value where to calculate the interpolation value.
    % 
    % RETURN
    % y ... Smooth interpolated value.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isscalar(edge0),'Input "edge0" must be scalar!');
    assert(isscalar(edge1),'Input "edge1" must be scalar!');
    assert(isscalar(x),'Input "x" must be scalar!');
    de = edge1 - edge0;
    y = 0.0;
    if((de > eps) || (de < -eps))
        x = (x - edge0) / de;
        x = min(max((x), 0.0), 1.0);
        y = x * x * (3.0 - x - x);
    else
        x = (x - edge0);
        if(x >= 0)
            y = 1.0;
        end
    end
end

