function targetPoint = DistancesTo3DPoint(distances, referencePoints)
    %ave.DistancesTo3DPoint Calculate the 3D position of a target point based on scalar distance measurements from at least four
    % reference points. The 3D position of M reference points must be known. They must span up an euclidean space, e.g.
    % 3 reference points making a triangle and the 4th reference point being outside the plane of that triangle. By
    % triangulation, the 3D position of a target point is calculated based on a variable number of distance measurements from
    % reference points to the target point.
    % 
    % INPUT
    % distances       ... N-by-M matrix of N distance measurements from all M reference points to the target point. Missing
    %                     distannce measurements may be indicated by NaN.
    % referencePoints ... 3-by-M matrix of M 3D points that span up an euclidean space. M must be at least 4, e.g. 3 points
    %                     making a triangle and the fourth point being outside the plane of that triangle.
    % 
    % OUTPUT
    % targetPoint ... 3-by-1 vector indicating the calculated target point that is "distances" away from "referencePoints".
    arguments (Input)
        distances (:,:) double
        referencePoints (3,:) double
    end
    arguments (Output)
        targetPoint (3,1) double
    end
    assert(size(distances,2) == size(referencePoints,2), 'Dimensions mismatch for distances and referencePoints!');
    N = size(distances, 1); % number of measurements
    M = size(distances, 2); % number of reference points
    assert(M > 3, 'There must be at least 4 reference points!');
    A = nan((M-1)*N, 3);
    b = nan((M-1)*N, 1);

    % go through all measurements
    k = 1;
    for n = 1:N
        % sort distances to make nonfinite values appear at the end
        d = distances(n,:);
        [d,i] = sort(d);
        p = referencePoints(:,i);

        % A matrix
        A(k:k+M-2,:) = 2.0 * (p(:,2:end) - p(:,1))';

        % b vector
        dd = d .* d;
        pp = sum(p .* p, 1);
        pp_diff = pp(2:end) - pp(1);
        dd_diff = dd(:,1) - dd(:,2:end);
        b(k:k+M-2) = pp_diff + dd_diff;

        % update index
        k = k + M - 1;
    end

    % remove all nonfinite measurements
    i = ~isfinite(b);
    if(any(i))
        A(i,:) = [];
        b(i) = [];
    end

    % solve
    targetPoint = A \ b;
end

