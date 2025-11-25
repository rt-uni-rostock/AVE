function indices = DouglasPeuckerSE2(poses, thresholdPosition, thresholdAngle)
    %DouglasPeuckerSE2 Run the douglas-peucker line simplification algorithm for a sequential list of poses [x;y;psi].
    % As long as the 2D position error is larger than the threshold, the algorithm behaves equal to the ave.geometry.DouglasPeucker2D algorithm.
    % If the position error of a simplified line segment is smaller than the specified thresholdPosition, then it is checked whether the angle error stays under the specified threshold.
    % If not, the line segment is further divided.
    % 
    % PARAMETER
    % poses ... 3-by-N matrix of N poses that represent the input path to be simplified.
    % thresholdPosition ... The maximum position error to be allowed for line simplification. This value must be positive.
    % thresholdAngle ... The maximum angular error to be allowed for line simplification. This value must be positive.
    % 
    % RETURN
    % indices ... One-based indices to those points indicating the simplified line. The output is empty if there are less than two input points.
    arguments (Input)
        poses (3,:) double
        thresholdPosition (1,1) double
        thresholdAngle (1,1) double
    end
    [indices_storage, indices_size] = SimplifyPath(poses, thresholdPosition, thresholdAngle);
    indices = indices_storage(1:indices_size) + int32(1);
end

function [indices_storage, indices_size] = SimplifyPath(poses, thresholdPosition, thresholdAngle)
    num_points = int32(size(poses, 2));
    indices_storage = zeros(num_points, 1, 'int32');
    indices_size = int32(0);

    % at least 2 points required
    if(num_points < int32(2)), return; end

    % first and last point must be in the list, the rest is obtained by recursive line simplification
    indices_size = int32(1);
    indices_storage(indices_size) = int32(0);
    [updated_indices_storage, updated_indices_size] = SimplifyPathRecursive(indices_storage, indices_size, poses, thresholdPosition, thresholdAngle, int32(0), num_points - int32(1));
    indices_storage = updated_indices_storage;
    indices_size = updated_indices_size;
    indices_size = indices_size + int32(1);
    indices_storage(indices_size) = num_points - int32(1);
end

function [updated_indices_storage, updated_indices_size] = SimplifyPathRecursive(indices_storage, indices_size, poses, thresholdPosition, thresholdAngle, idxStart, idxEnd)
    updated_indices_storage = indices_storage;
    updated_indices_size = indices_size;

    % just a line: nothing to do
    if((idxStart + int32(1)) >= idxEnd), return; end

    % implicit edge function
    xStart = poses(1, idxStart + int32(1));
    yStart = poses(2, idxStart + int32(1));
    xEnd = poses(1, idxEnd + int32(1));
    yEnd = poses(2, idxEnd + int32(1));
    a = yEnd - yStart;
    b = xStart - xEnd;
    len = sqrt(a*a + b*b);
    ip = int32(0);
    dmax = -1.0;
    amax = -1.0;
    if(len > 0.0)
        len = 1.0 / len;
        a = a * len;
        b = b * len;
        c = -a * xStart - b * yStart;

        % search farthest point for position and largest angle error
        for i = (idxStart + int32(1)):(idxEnd - int32(1))
            len = abs(a*poses(1, i + int32(1)) + b*poses(2, i + int32(1)) + c);
            amax = max(amax, abs(ave.SymmetricalAngle(poses(3, i + int32(1)) - poses(3, idxStart + int32(1)))));
            if(len > dmax)
                dmax = len;
                ip = i;
            end
        end
    else
        % search farthest point for position and largest angle error
        for i = (idxStart + int32(1)):(idxEnd - int32(1))
            xEnd = poses(1, i + int32(1)) - xStart;
            yEnd = poses(2, i + int32(1)) - yStart;
            len = xEnd*xEnd + yEnd*yEnd;
            amax = max(amax, abs(ave.SymmetricalAngle(poses(3, i + int32(1)) - poses(3, idxStart + int32(1)))));
            if(len > dmax)
                dmax = len;
                ip = i;
            end
        end
    end

    % if position error is too large, divide according to position
    if(dmax >= thresholdPosition)
        [out_storage, out_size] = SimplifyPathRecursive(updated_indices_storage, updated_indices_size, poses, thresholdPosition, thresholdAngle, idxStart, ip);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;

        updated_indices_size = updated_indices_size + int32(1);
        updated_indices_storage(updated_indices_size) = ip;

        [out_storage, out_size] = SimplifyPathRecursive(updated_indices_storage, updated_indices_size, poses, thresholdPosition, thresholdAngle, ip, idxEnd);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;
    % else if angle error is too large, divide remaining interval
    elseif(amax >= thresholdAngle)
        ia = idxStart + int32(floor((idxEnd - idxStart) / 2));
        [out_storage, out_size] = SimplifyPathRecursive(updated_indices_storage, updated_indices_size, poses, thresholdPosition, thresholdAngle, idxStart, ia);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;

        updated_indices_size = updated_indices_size + int32(1);
        updated_indices_storage(updated_indices_size) = ia;

        [out_storage, out_size] = SimplifyPathRecursive(updated_indices_storage, updated_indices_size, poses, thresholdPosition, thresholdAngle, ia, idxEnd);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;
    end
end
