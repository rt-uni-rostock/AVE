function indices = DouglasPeucker2D(points, threshold)
    %DouglasPeucker2D Run the douglas-peucker line simplification algorithm for a sequential list of 2-dimensional points.
    % 
    % PARAMETER
    % points ... 2-by-N matrix of N points that represent the input line to be simplified.
    % threshold ... The maximum error to be allowed for line simplification. This value must be positive.
    % 
    % RETURN
    % indices ... One-based indices to those points indicating the simplified line. The output is empty if there are less than two input points.
    arguments (Input)
        points (2,:) double
        threshold (1,1) double
    end
    [indices_storage, indices_size] = SimplifyLine(points, threshold);
    indices = indices_storage(1:indices_size) + int32(1);
end

function [indices_storage, indices_size] = SimplifyLine(points, threshold)
    num_points = int32(size(points, 2));
    indices_storage = zeros(num_points, 1, 'int32');
    indices_size = int32(0);

    % at least 2 points required
    if(num_points < int32(2)), return; end

    % first and last point must be in the list, the rest is obtained by recursive line simplification
    indices_size = int32(1);
    indices_storage(indices_size) = int32(0);
    [updated_indices_storage, updated_indices_size] = SimplifyLineRecursive(indices_storage, indices_size, points, threshold, int32(0), num_points - int32(1));
    indices_storage = updated_indices_storage;
    indices_size = updated_indices_size;
    indices_size = indices_size + int32(1);
    indices_storage(indices_size) = num_points - int32(1);
end

function [updated_indices_storage, updated_indices_size] = SimplifyLineRecursive(indices_storage, indices_size, points, threshold, idxStart, idxEnd)
    updated_indices_storage = indices_storage;
    updated_indices_size = indices_size;

    % just a line: nothing to do
    if((idxStart + int32(1)) >= idxEnd), return; end

    % implicit edge function
    xStart = points(1, idxStart + int32(1));
    yStart = points(2, idxStart + int32(1));
    xEnd = points(1, idxEnd + int32(1));
    yEnd = points(2, idxEnd + int32(1));
    a = yEnd - yStart;
    b = xStart - xEnd;
    len = sqrt(a*a + b*b);
    imax = int32(0);
    dmax = -1.0;
    if(len > 0.0)
        len = 1.0 / len;
        a = a * len;
        b = b * len;
        c = -a * xStart - b * yStart;

        % search farthest point
        for i = (idxStart + int32(1)):(idxEnd - int32(1))
            len = abs(a*points(1, i + int32(1)) + b*points(2, i + int32(1)) + c);
            if(len > dmax)
                dmax = len;
                imax = i;
            end
        end
    else
        % search farthest point
        for i = (idxStart + int32(1)):(idxEnd - int32(1))
            xEnd = points(1, i + int32(1)) - xStart;
            yEnd = points(2, i + int32(1)) - yStart;
            len = xEnd*xEnd + yEnd*yEnd;
            if(len > dmax)
                dmax = len;
                imax = i;
            end
        end
    end

    % check threshold and divide
    if(dmax >= threshold)
        [out_storage, out_size] = SimplifyLineRecursive(updated_indices_storage, updated_indices_size, points, threshold, idxStart, imax);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;

        updated_indices_size = updated_indices_size + int32(1);
        updated_indices_storage(updated_indices_size) = imax;

        [out_storage, out_size] = SimplifyLineRecursive(updated_indices_storage, updated_indices_size, points, threshold, imax, idxEnd);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;
    end
end
