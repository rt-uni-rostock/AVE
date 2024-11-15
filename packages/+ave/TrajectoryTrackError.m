function [valid, trackError, closestSegmentIndices, closestSegmentLinearInterpolationRatio, targetSegmentIndices, targetSegmentLinearInterpolationRatio] = TrajectoryTrackError(time, positionLLA, matTimeLatLon, numPoints)
    %ave.TrajectoryTrackError Calculate the track error to a given trajectory and get the indices and interpolation value for the closest point and the target point.
    % All input positions are transformed to an euclidian NED frame. Therefore, this function may not be used if the points of the trajectory are too far apart.
    % 
    % 
    % --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    % PARAMETERS
    % --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    % time          ... [double x 1] The current time value to be compared with the trajectory. If the time is not within the range of the trajectory time window, then the valid output remains false (invalid).
    % positionLLA   ... [double x 3] The geographic position of the vehicle for which to calculate the distance to the trajectory. The elements are
    %                                1: latitude (radians)
    %                                2: longitude (radians)
    %                                3: altitude (meters, positive upwards)
    % matTimeLatLon ... [double x 3 x N] 3-by-N matrix representing the time and 2D geographic position data of the trajectory. Each column indicates one point of the trajectory. The format of one column is as follows:
    %                                1: time (seconds), must be monotonically increasing
    %                                2: latitude (radians)
    %                                3: longitude (radians)
    % numPoints     ... [int32 x 1] The actual number of columns from the matTimeLatLon matrix to be considered for the calculation.
    % 
    % 
    % --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    % RETURN
    % --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    % valid                                  ... [boolean x 1] True if all outputs are valid and the computation was successful, false otherwise. The output is invalid, if the input time is not within the
    %                                                          time range of the trajectory, if the number of points is less than 2 or some numerical problems occured (e.g. two points were to close to compute a line).
    % trackError                             ... [double x 2] 2-dimensional vector containing track errors xb (along the track) and yb (cross track error). The elements are as follows
    %                                                         1: xb, error from the closest point to the trajectory to the target point of the trajectory (according to given input time). This value is an accumulation of all line segments in between.
    %                                                            xb is positive if the target point is "after" the closest point.
    %                                                         2: yb, lateral distance to the closest line segment of the trajectory. yb is positive if the vehicle is "on the right" of the trajectory.
    %                                                            If the vehicle is "before" or "after" the trajectory, then an extrapolated point according to the first or last line segment is calculated. xb and yb then take the distance to this
    %                                                            extrapolated line into account.
    % closestSegmentIndices                  ... [int32 x 2] The two indices of the line segment that is closest to the vehicle position. If the output is not valid, these values are undefined.
    % closestSegmentLinearInterpolationRatio ... [double x 1] The scalar interpolation ratio for the closest line segment. The indices of the line segment indicate two values v1 and v2. The interpolated value is calculated by v = v1 + ratio*(v2 - v1).
    % targetSegmentIndices                   ... [int32 x 2] The two indices of the line segment that contain the target position (according to the input time). If the output is not valid, these values are undefined.
    % targetSegmentLinearInterpolationRatio  ... [double x 1] The scalar interpolation ratio for the target line segment. The indices of the line segment indicate two values v1 and v2. The interpolated value is calculated by v = v1 + ratio*(v2 - v1).
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221101    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Ensure correct inputs
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isscalar(time) && isa(time, 'double'), 'Input "time" must be a scalar of type double!');
    assert((3 == numel(positionLLA)) && isa(positionLLA, 'double'), 'Input "positionLLA" must be a 3-dimensional vector of type double!');
    assert(isa(matTimeLatLon, 'double') && (3 == size(matTimeLatLon,1)), 'Input "matTimeLatLon" must be a 3-by-N matrix of type double!');
    assert(isscalar(numPoints), 'Input "numPoints" must be scalar!');

    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Fallback output
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    valid = false;
    trackError = zeros(2,1);
    closestSegmentIndices = int32([-1; -1]);
    closestSegmentLinearInterpolationRatio = 0.0;
    targetSegmentIndices = int32([-1; -1]);
    targetSegmentLinearInterpolationRatio = 0.0;

    % If there're not enough points we return without a valid result
    numPoints = int32(min(numPoints, size(matTimeLatLon,2)));
    if(numPoints < int32(2)), return; end

    % If the input time is out of the time range of the trajectory we return without a valid result
    if((time < matTimeLatLon(1,1)) || (time > matTimeLatLon(1,numPoints))), return; end


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Transform lat,lon from matrix to euclidian posNorth, posEast
    % The vehicle position (positionLLA) is the origin
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    [posNorth, posEast, ~] = ave.LLA2NED(matTimeLatLon(2,int32(1):numPoints), matTimeLatLon(3,int32(1):numPoints), repmat(positionLLA(3), [numPoints int32(1)]), positionLLA(1), positionLLA(2), positionLLA(3));


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Find closest point to path
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    [idx1, idx2, interpolation, distance] = GetClosestPoint(posNorth,posEast,numPoints);
    if((idx1 < int32(1)) || (idx2 < int32(1))), return; end
    closestSegmentIndices = [idx1; idx2];
    closestSegmentLinearInterpolationRatio = interpolation;
    P1 = [posNorth(idx1); posEast(idx1)];
    P2 = [posNorth(idx2); posEast(idx2)];


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Track error
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Sign of yb: negative if left of trajectory, positive otherwise
    c = cross([-P1; 0], [P2-P1; 0]);
    sign_yb = sign(c(3));

    % Crosstrack error (yb) is closest distance to path.
    trackError(2) = sign_yb * distance;

    % By default the closest point is on a line segment and we can initialize the trackerror (xb) by the distance to P1
    closestInterpolatedPoint = P1 + interpolation * (P2 - P1);
    trackError(1) = -sqrt((closestInterpolatedPoint - P1)' * (closestInterpolatedPoint - P1));
    idxStart = idx1;

    % However, if we are just closest to the first or last point, we extend the respective line segment and recalculate the crosstrack error (yb)
    % The closest point for trackerror calculation is an extrapolated point
    if(((idx1 == 1) && (interpolation < 1e-12)) || ((idx2 == numPoints) && (interpolation > (1 - 1e-12))))
        % Calculate the closest extrapolated point
        N = P2 - P1;
        NN = N' * N;
        if(abs(NN) < 1e-12), return; end
        lambda = -(P1' * N) / NN;
        closestExtrapolatedPoint = P1 + lambda * N;

        % The trackerror (yb) is the distance to this extrapolated point
        c = cross([-closestExtrapolatedPoint; 0], [P2-P1; 0]);
        sign_yb = sign(c(3));
        trackError(2) = sign_yb * sqrt(closestExtrapolatedPoint' * closestExtrapolatedPoint);

        % The trackerror along the track (xb) is initialized with the distance from the extrapolated point to the first/last point of the trajectory
        if((idx1 == 1) && (interpolation < 1e-12))
            % Extrapolated point is "before" trajectory: add distance from extrapolated point to P1
            trackError(1) = sqrt((P1 - closestExtrapolatedPoint)' * (P1 - closestExtrapolatedPoint));
            idxStart = idx1;
        else
            % Extrapolated point is "after" trajectory: substract distance from extrapolated point to P2
            trackError(1) = -sqrt((closestExtrapolatedPoint - P2)' * (closestExtrapolatedPoint - P2));
            idxStart = idx2;
        end
    end

    % Now the trackerror (xb) relates to the point indexed by idxXStart
    % We need to accumulate the the distance from this point to the target point which corresponds to the time input
    % Find the first index that is greater than the time
    idxEnd = int32(find(matTimeLatLon(1,int32(1):numPoints) > time, 1));
    if(isempty(idxEnd)), return; end
    idxEnd = idxEnd(1); % <-- required to make simulink happy
    if(idxEnd < 2), return; end

    % First, we compute the distance from idxStart to idxEnd and add it to the trackError (xb)
    idxA = int32(min(idxStart,idxEnd));
    idxB = int32(max(idxStart,idxEnd));
    sign_xb = 1;
    if(idxStart > idxEnd), sign_xb = -1; end
    for i = (idxA + int32(1)):idxB
        pA = [posNorth(i - int32(1)); posEast(i - int32(1))];
        pB = [posNorth(i); posEast(i)];
        trackError(1) = trackError(1) + sign_xb * sqrt((pB - pA)' * (pB - pA));
    end

    % Finally, we compute the distance from the endpoint (idxEnd) to the target time (which is always "before" the endpoint (idxEnd))
    idxPrevEnd = idxEnd - int32(1);
    timeEnd = matTimeLatLon(1,idxEnd);
    timePrevEnd = matTimeLatLon(1,idxPrevEnd);
    timeSegment = timeEnd - timePrevEnd;
    if(timeSegment < 1e-12), return; end
    ratio = (time - timePrevEnd) / timeSegment;
    pointEnd = [posNorth(idxEnd); posEast(idxEnd)];
    pointPrevEnd = [posNorth(idxPrevEnd); posEast(idxPrevEnd)];
    pointT = pointPrevEnd + ratio * (pointEnd - pointPrevEnd);
    trackError(1) = trackError(1) - sqrt((pointEnd - pointT)' * (pointEnd - pointT));
    targetSegmentIndices = int32([idxPrevEnd; idxEnd]);
    targetSegmentLinearInterpolationRatio = ratio;

    % We made it: return success
    valid = true;
end


function [idx1, idx2, interpolation, distance] = GetClosestPoint(posNorth,posEast,numPoints)
    idx1 = int32(-1);
    idx2 = int32(-1);
    distance = 0;
    interpolation = 0;
    if(numPoints < int32(2))
        return;
    end
    idx1 = int32(1);
    idx2 = int32(2);
    minSquaredDistance = inf;
    for n = int32(2):numPoints
        P1 = [posNorth(n - int32(1)); posEast(n - int32(1))];
        P2 = [posNorth(n); posEast(n)];
        N = P2 - P1;
        NN = N' * N;
        squaredDistance = P1' * P1;
        lambda = 0;
        if(abs(NN) > 1e-12)
            lambda = min(max(-(P1' * N) / NN, 0), 1);
            squaredDistance = (P1 + lambda*N)' * (P1 + lambda*N);
        end
        if(squaredDistance < minSquaredDistance)
            minSquaredDistance = squaredDistance;
            distance = sqrt(squaredDistance);
            interpolation = lambda;
            idx1 = n - int32(1);
            idx2 = n;
        end
    end
end

