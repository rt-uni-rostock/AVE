function indices = QuickHull2D(refPoints)
    %QuickHull2D Compute the convex hull for a set of 2D points using the QuickHull algorithm.
    % 
    % PARAMETER
    % refPoints ... 2-by-N matrix of N points for which to compute the convex hull.
    % 
    % RETURN
    % indices ... One-based indices to input points that form the convex hull. The output will be empty if the number of 2D points is less than three!
    [indices_storage, indices_size] = ComputeHull(refPoints);
    indices = indices_storage(1:indices_size) + int32(1);
end

function [indices_storage, indices_size] = ComputeHull(refPoints)
    num_points = int32(size(refPoints, 2));
    indices_storage = zeros(num_points, 1, 'int32');
    indices_size = int32(0);

    % at least 3 points required
    if(num_points < int32(3)), return; end

    % find min/max points
    idxMin = int32(0);
    idxMax = int32(0);
    for i = int32(1):(num_points - int32(1))
        if(refPoints(1, i + int32(1)) < refPoints(1, idxMin + int32(1)))
            idxMin = i;
        elseif(refPoints(1, i + int32(1)) > refPoints(1, idxMax + int32(1)))
            idxMax = i;
        end
    end

    % edge of min/max points
    a = refPoints(2, idxMax + int32(1)) - refPoints(2, idxMin + int32(1));
    b = refPoints(1, idxMin + int32(1)) - refPoints(1, idxMax + int32(1));
    c = -a * refPoints(1, idxMin + int32(1)) - b * refPoints(2, idxMin + int32(1));

    % divide points in two segments positive (P) and negative (N)
    indicesPos = zeros(num_points, 1, 'int32');
    indicesNeg = zeros(num_points, 1, 'int32');
    kp = int32(0);
    kn = int32(0);
    idxPosFarthest = int32(-1);
    idxNegFarthest = int32(-1);
    dPosFarthest = 0.0;
    dNegFarthest = 0.0;
    for k = int32(0):(num_points - int32(1))
        if((k == idxMin) || (k == idxMax)), continue; end
        d = a * refPoints(1, k + int32(1)) + b * refPoints(2, k + int32(1)) + c;
        if(d < 0.0)
            kn = kn + int32(1);
            indicesNeg(kn) = k;
            if(d < dNegFarthest)
                dNegFarthest = d;
                idxNegFarthest = k;
            end
        else
            kp = kp + int32(1);
            indicesPos(kp) = k;
            if(d > dPosFarthest)
                dPosFarthest = d;
                idxPosFarthest = k;
            end
        end
    end

    % find convex hull for both sets
    indices_size = indices_size + int32(1);
    indices_storage(indices_size) = idxMin;
    if(idxPosFarthest >= 0)
        [updated_indices_storage, updated_indices_size] = ComputeHullRecursive(indices_storage, indices_size, refPoints, indicesPos(1:kp), idxMin, idxMax, idxPosFarthest);
        indices_storage = updated_indices_storage;
        indices_size = updated_indices_size;
    end
    indices_size = indices_size + int32(1);
    indices_storage(indices_size) = idxMax;
    if(idxNegFarthest >= 0)
        [updated_indices_storage, updated_indices_size] = ComputeHullRecursive(indices_storage, indices_size, refPoints, indicesNeg(1:kn), idxMax, idxMin, idxNegFarthest);
        indices_storage = updated_indices_storage;
        indices_size = updated_indices_size;
    end

    % convexity check, remove collinear points
    if(indices_size < int32(3))
        indices_size = int32(0);
    else
        deps = 100.0 * eps;
        keep = zeros(indices_size, 1, 'int32');
        kk = int32(0);
        prevNormalX = refPoints(2, indices_storage(1) + int32(1)) - refPoints(2, indices_storage(indices_size) + int32(1));
        prevNormalY = refPoints(1, indices_storage(indices_size) + int32(1)) - refPoints(1, indices_storage(1) + int32(1));
        len = sqrt(prevNormalX*prevNormalX + prevNormalY*prevNormalY);
        len = 1.0 / (len + double(len == 0.0));
        prevNormalX = prevNormalX * len;
        prevNormalY = prevNormalY * len;
        for i = int32(0):(indices_size - int32(1))
            iNext = mod((i + int32(1)), indices_size);
            curNormalX = refPoints(2, indices_storage(iNext + int32(1)) + int32(1)) - refPoints(2, indices_storage(i + int32(1)) + int32(1));
            curNormalY = refPoints(1, indices_storage(i + int32(1)) + int32(1)) - refPoints(1, indices_storage(iNext + int32(1)) + int32(1));
            len = sqrt(curNormalX*curNormalX + curNormalY*curNormalY);
            len = 1.0 / (len + double(len == 0.0));
            curNormalX = curNormalX * len;
            curNormalY = curNormalY * len;
            if(abs(prevNormalX * curNormalY - prevNormalY * curNormalX) > deps)
                kk = kk + int32(1);
                keep(kk) = indices_storage(i + int32(1));
                prevNormalX = curNormalX;
                prevNormalY = curNormalY;
            end
        end
        indices_storage(int32(1):kk) = keep(int32(1):kk);
        indices_size = kk;
        if(indices_size < int32(3))
            indices_size = int32(0);
        end
    end
end

function [updated_indices_storage, updated_indices_size] = ComputeHullRecursive(indices_storage, indices_size, refPoints, idxPoints, idxA, idxB, idxFarthest)
    updated_indices_storage = indices_storage;
    updated_indices_size = indices_size;

    % terminate if no points in the list
    N = int32(numel(idxPoints));
    if(~N), return; end

    % edge from A to farthest
    aEdgeA = refPoints(2, idxFarthest + int32(1)) - refPoints(2, idxA + int32(1));
    bEdgeA = refPoints(1, idxA + int32(1)) - refPoints(1, idxFarthest + int32(1));
    cEdgeA = -aEdgeA*refPoints(1, idxA + int32(1)) - bEdgeA*refPoints(2, idxA + int32(1));

    % edge from farthest to B
    aEdgeB = refPoints(2, idxB + int32(1)) - refPoints(2, idxFarthest + int32(1));
    bEdgeB = refPoints(1, idxFarthest + int32(1)) - refPoints(1, idxB + int32(1));
    cEdgeB = -aEdgeB*refPoints(1, idxFarthest + int32(1)) - bEdgeB*refPoints(2, idxFarthest + int32(1));

    % use points on positive side only
    idxPointsA = zeros(N, 1, 'int32');
    idxPointsB = zeros(N, 1, 'int32');
    ka = int32(0);
    kb = int32(0);
    dmaxA = 0.0;
    dmaxB = 0.0;
    idxFarthestA = int32(-1);
    idxFarthestB = int32(-1);
    for k = int32(0):(N - int32(1))
        if(idxFarthest == idxPoints(k + int32(1))), continue; end
        dA = aEdgeA * refPoints(1, idxPoints(k + int32(1)) + int32(1)) + bEdgeA * refPoints(2, idxPoints(k + int32(1)) + int32(1)) + cEdgeA;
        dB = aEdgeB * refPoints(1, idxPoints(k + int32(1)) + int32(1)) + bEdgeB * refPoints(2, idxPoints(k + int32(1)) + int32(1)) + cEdgeB;
        if(dA > 0)
            ka = ka + int32(1);
            idxPointsA(ka) = idxPoints(k + int32(1));
            if(dA > dmaxA)
                dmaxA = dA;
                idxFarthestA = idxPoints(k + int32(1));
            end
        end
        if(dB > 0)
            kb = kb + int32(1);
            idxPointsB(kb) = idxPoints(k + int32(1));
            if(dB > dmaxB)
                dmaxB = dB;
                idxFarthestB = idxPoints(k + int32(1));
            end
        end
    end

    % find hulls recursively
    if(idxFarthestA >= 0)
        [out_storage, out_size] = ComputeHullRecursive(updated_indices_storage, updated_indices_size, refPoints, idxPointsA(1:ka), idxA, idxFarthest, idxFarthestA);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;
    end
    updated_indices_size = updated_indices_size + int32(1);
    updated_indices_storage(updated_indices_size) = idxFarthest;
    if(idxFarthestB >= 0)
        [out_storage, out_size] = ComputeHullRecursive(updated_indices_storage, updated_indices_size, refPoints, idxPointsB(1:kb), idxFarthest, idxB, idxFarthestB);
        updated_indices_storage = out_storage;
        updated_indices_size = out_size;
    end
end
