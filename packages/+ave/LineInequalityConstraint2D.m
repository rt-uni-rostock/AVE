function [A, b] = LineInequalityConstraint2D(point1, point2)
    % Generate a 2D line inequality constraint of the form A*p <= b, where p is a two-dimensional point.
    % The line, that divides the space into two half planes, is defined by the two input points. The
    % inequality is true for all points that are in the direction of cross([0;0;1],[point2 - point1; 0])
    % starting from any point on the line.
    %
    %    x ^           point2
    %      |             O
    %      |           .
    %      |         .
    %      |       .
    %      |     .       A*[x;y] <= b
    %      |   O        satisfied here
    %      | point1
    %      +-------------------------->
    %                                 y
    % If the two given points are too close to each other, then {A,b} will be an infeasible constraint with
    % A = [0 0] and b = -1.
    % 
    % 
    % PARAMETERS
    % point1 ... Two-dimensional point that lies on the line dividing the space into two half planes.
    % point2 ... A second two-dimensional point that lies on the line.
    % 
    % 
    % RETURN
    % A ... 1-by-2 matrix for the linear inequality constraint A*p <= b.
    % b ... Scalar value for the linear inequality constraint A*p <= b.
    assert(isa(point1, 'double') && (2 == numel(point1)), 'Invalid data type or dimension of "point1"!');
    assert(isa(point2, 'double') && (2 == numel(point2)), 'Invalid data type or dimension of "point2"!');
    dx = point2(1) - point1(1);
    dy = point2(2) - point1(2);
    L = sqrt(dx*dx + dy*dy);
    A = [0 0];
    b = -1;
    if(L > 1e-12)
        A = [dy, -dx] / L;
        b = A * [point1(1); point1(2)];
    end
end

