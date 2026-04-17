function targetPoint = DistancesTo3DPoint(distances, referencePoints)
    %ave.DistancesTo3DPoint Calculate the 3D position of a target point based on scalar distance measurements from at least four
    % reference points. The 3D position of M reference points must be known. They must span up an euclidean space, e.g.
    % 3 reference points making a triangle and the 4th reference point being outside the plane of that triangle. By
    % triangulation, the 3D position of a target point is calculated based on a variable number of distance measurements from
    % reference points to the target point.
    % 
    % INPUT
    % distances       ... N-by-M matrix of N distance measurements from all M reference points to the target point.
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

    % A matrix
    p_diff_2 = 2.0 * (referencePoints(:,2:end) - referencePoints(:,1));
    A = reshape(repmat(p_diff_2, [N,1]), 3, [])';

    % b vector
    distances_squared = distances .* distances;
    pp = sum(referencePoints .* referencePoints, 1);
    pp_diff = pp(2:end) - pp(1);
    sd_diff = distances_squared(:,1) - distances_squared(:,2:end);
    b = reshape(repmat(pp_diff, [N,1]), [], 1) + reshape(sd_diff, [], 1);

    % solve
    targetPoint = A \ b;
end

