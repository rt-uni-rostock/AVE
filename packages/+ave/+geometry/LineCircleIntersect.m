function s = LineCircleIntersect(pointA, pointB, center, radius)
    %ave.geometry.LineCircleIntersect Calculate the intersection of a line with a circle.
    % 
    % PARAMETER
    % pointA ... 2-D start point of the line from point A to point B.
    % pointB ... 2-D end point of the line from point A to point B.
    % center ... 2-D center position of the circle.
    % radius ... Scalar radius of the circle.
    % 
    % RETURN
    % s ... 2-by-1 vector indicating the ratio where the line intersects the circle. An intersection point is calculated
    %       by p = pointA + s(i) * (pointB - pointA), for i = 1,2. For more information, take a look to 'help ave.SolveQuadraticEquation'.
    arguments
        pointA (2,1) double
        pointB (2,1) double
        center (2,1) double
        radius (1,1) double
    end
    u = pointA - center;
    v = pointB - center;
    a = (v - u)' * (v - u);
    b = 2.0 * u' * (v - u);
    c = u' * u - radius*radius;
    s = ave.SolveQuadraticEquation(a, b, c);
end
