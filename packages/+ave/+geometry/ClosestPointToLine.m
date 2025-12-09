function [p, lambda] = ClosestPointToLine(point, p0, p1)
    %ave.geometry.ClosestPointToLine Calculate the closest point to a line.
    % 
    % PARAMETER
    % point ... 2-D euclidean point from where to calculate the closest point to the line.
    % p0 ... Start point of the line.
    % p1 ... End point of the line.
    % 
    % RETURN
    % p ... A point on the line from p0 to p1 that is closest to the given point.
    % lambda ... Linear interpolation ratio in range [0,1] that indicates the closest point according to p = p0 + lambda*(p1 - p0).
    arguments
        point (2,1) double
        p0 (2,1) double
        p1 (2,1) double
    end
    d = p1 - p0;
    v = point - p0;
    s = (d' * d);
    lambda = 0;
    if(s > 0.0)
        lambda = (d' * v) / s;
    end
    lambda = min(max(lambda, 0.0), 1.0);
    p = p0 + lambda * d;
end
