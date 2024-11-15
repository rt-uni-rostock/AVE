function clampedValue = Clamp(value, lowerBound, upperBound)
    %ave.Clamp Clamp a value to the range [lowerBound, upperBound] or [upperBound, lowerBound] if lowerBound > upperBound.
    % 
    % PARAMETERS
    % value      ... The input value to clamp.
    % lowerBound ... The lower boundary value.
    % upperBound ... The upper boundary value.
    % 
    % RETURN
    % clampedValue ... The input value clamped to the range [lowerBound, upperBound] or [upperBound, lowerBound] if lowerBound > upperBound.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20201028    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    lo = min(lowerBound,upperBound);
    hi = max(lowerBound,upperBound);
    clampedValue = max(lo,min(value,hi));
end

