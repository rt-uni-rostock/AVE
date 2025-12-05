classdef ConvexPolygon < handle
    methods
        function this = ConvexPolygon(maxNumVertices)
            % Construct a new convex polygon. Internally, a constant-size memory is used to allow code generation for cell-arrays of convex polygons.
            % 
            % PARAMETER
            % maxNumVertices ... The maximum number of vertices that can be stored for this convex polygon.
            arguments (Input)
                maxNumVertices (1,1) uint32
            end

            % allocate memory
            this.num_vertices = int32(0);
            this.vertices = zeros(2, maxNumVertices);
            this.edges = zeros(3, maxNumVertices);
            this.aabb_lowerBound = zeros(2, 1);
            this.aabb_upperBound = zeros(2, 1);
        end
        function Create(this, vertices)
            % Create the convex polygon based on vertices.
            % IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT!
            % 
            % PARAMETER
            % vertices ... 2-by-N matrix of N vertices.
            arguments (Input)
                this ave.geometry.ConvexPolygon
                vertices (2,:) double
            end

            % copy to internal vertices (replace non-finite values by zero)
            this.num_vertices = min(int32(size(vertices,2)), int32(size(this.vertices,2)));
            for k = int32(1):this.num_vertices
                if(isfinite(vertices(1, k)) && isfinite(vertices(2, k)))
                    this.vertices(:, k) = vertices(:, k);
                else
                    this.vertices(:, k) = zeros(2, 1);
                end
            end

            % at least 3 vertices are required
            if(this.num_vertices < 3)
                % not enough vertices: set default values
                this.aabb_lowerBound(:) = 0.0;
                this.aabb_upperBound(:) = 0.0;
                this.edges(:) = 0.0;
            else
                this.aabb_lowerBound(:) = this.vertices(:,1);
                this.aabb_upperBound(:) = this.vertices(:,1);
                for k = int32(2):this.num_vertices
                    this.RebuildEdge(k - int32(1), k);
                    this.aabb_lowerBound(1) = min(this.aabb_lowerBound(1), this.vertices(1,k));
                    this.aabb_lowerBound(2) = min(this.aabb_lowerBound(2), this.vertices(2,k));
                    this.aabb_upperBound(1) = max(this.aabb_upperBound(1), this.vertices(1,k));
                    this.aabb_upperBound(2) = max(this.aabb_upperBound(2), this.vertices(2,k));
                end
                this.RebuildEdge(this.num_vertices, 1);
            end
        end
        function isConvex = IsConvex(this)
            % Check if all edges represent a convex polygon of correct vertex order.
            % 
            % RETURN
            % isConvex ... True if polygon is convex, false otherwise. The polygon is convex if the order of vertices is correct (positive rotation around z) and there are no duplicated points or multiple rings or spirals of edges.
            arguments (Input)
                this ave.geometry.ConvexPolygon
            end

            % there must be at least 3 vertices
            isConvex = false;
            if(this.num_vertices < int32(3))
                return;
            end

            % go through all vertices and check correct order
            numSignChangesX = int32(0);
            numSignChangesY = int32(0);
            deps = 100.0 * eps;
            for k = int32(0):int32(1):(this.num_vertices - int32(1))
                kPrev = mod(k + this.num_vertices - int32(1), this.num_vertices);
                kNext = mod(k + int32(1), this.num_vertices);
                v0 = this.vertices(:, kPrev + int32(1));
                v1 = this.vertices(:, k + int32(1));
                v2 = this.vertices(:, kNext + int32(1));
                dx1 = v1(1) - v0(1);
                dy1 = v1(2) - v0(2);
                L1 = sqrt(dx1*dx1 + dy1*dy1);
                sx1 = int32(sign(dx1)) >= int32(0);
                sy1 = int32(sign(dy1)) >= int32(0);
                dx1 = dx1 / L1;
                dy1 = dy1 / L1;
                dx2 = v2(1) - v1(1);
                dy2 = v2(2) - v1(2);
                L2 = sqrt(dx2*dx2 + dy2*dy2);
                sx2 = int32(sign(dx2)) >= int32(0);
                sy2 = int32(sign(dy2)) >= int32(0);
                dx2 = dx2 / L2;
                dy2 = dy2 / L2;
                z = dx1*dy2 - dy1*dx2;
                numSignChangesX = numSignChangesX + int32(sx1 ~= sx2);
                numSignChangesY = numSignChangesY + int32(sy1 ~= sy2);
                if((L1 <= deps) || (L2 <= deps) || (z <= deps))
                    return;
                end
            end
            isConvex = (int32(2) == numSignChangesX) && (int32(2) == numSignChangesY);
        end
        function isInside = IsInside(this, point)
            % Check whether a point is inside the convex polygon or not.
            % IMPORTANT: IT IS NOT CHECKED WHETHER THE POLYGON IS CONVEX OR NOT!
            % 
            % PARAMETER
            % point ... The query point that must contain at least 2 elements.
            % 
            % RETURN
            % isInside ... True if point is inside the convex polygon (closed set, including borders), false otherwise.
            arguments (Input)
                this ave.geometry.ConvexPolygon
                point (2,1) double
            end
            outside = false;
            for k = int32(1):this.num_vertices
                outside = outside || ((this.edges(1, k) * point(1) + this.edges(2, k) * point(2) + this.edges(3, k)) > 0.0);
            end
            isInside = ~outside;
        end
        function Transform(this, x, y, cosPsi, sinPsi)
            % Transform all vertices of the convex polygon. The transformation is specified by a rotation around the origin {0,0} followed by a translation.
            % This member function has no effect, if the number of vertices is lower than 3!
            % 
            % PARAMETER
            % x ... Translation in the x direction.
            % y ... Translation in the y direction.
            % cosPsi ... The cosine of rotation angle psi.
            % sinPsi ... The sine of rotation angle psi.
            arguments (Input)
                this ave.geometry.ConvexPolygon
                x (1,1) double
                y (1,1) double
                cosPsi (1,1) double
                sinPsi (1,1) double
            end
            if(this.num_vertices > int32(2))
                vx = this.vertices(1,1);
                vy = this.vertices(2,1);
                this.vertices(1,1) = x + cosPsi * vx - sinPsi * vy;
                this.vertices(2,1) = y + sinPsi * vx + cosPsi * vy;
                this.aabb_lowerBound(:) = this.vertices(:,1);
                this.aabb_upperBound(:) = this.vertices(:,1);
                for k = int32(2):this.num_vertices
                    vx = this.vertices(1,k);
                    vy = this.vertices(2,k);
                    this.vertices(1,k) = x + cosPsi * vx - sinPsi * vy;
                    this.vertices(2,k) = y + sinPsi * vx + cosPsi * vy;
                    this.RebuildEdge(k - 1, k);
                    this.aabb_lowerBound(1) = min(this.aabb_lowerBound(1), this.vertices(1,k));
                    this.aabb_lowerBound(2) = min(this.aabb_lowerBound(2), this.vertices(2,k));
                    this.aabb_upperBound(1) = max(this.aabb_upperBound(1), this.vertices(1,k));
                    this.aabb_upperBound(2) = max(this.aabb_upperBound(2), this.vertices(2,k));
                end
                this.RebuildEdge(this.num_vertices, 1);
            end
        end
        function minSquaredDistance = MinimumSquaredDistanceToEdges(this, point)
            % Calculate the minimum squared distance to all edges of this convex polygon.
            % IMPORTANT: IT IS NOT CHECKED WHETHER THIS POLYGON IS VALID OR NOT!
            % 
            % PARAMETER
            % point ... Query point for which to calculate the minimum squared distance.
            % 
            % RETURN
            % minSquaredDistance ... The minimum squared distance to all edges of this convex polygon or infinity, if this convex polygon contains less than 3 vertices.
            arguments (Input)
                this ave.geometry.ConvexPolygon
                point (2,1) double
            end
            minSquaredDistance = inf;
            if(this.num_vertices < int32(3))
                return;
            end
            ax = point(1) - this.vertices(1, this.num_vertices);
            ay = point(2) - this.vertices(2, this.num_vertices);
            bx = this.vertices(1,1) - this.vertices(1, this.num_vertices);
            by = this.vertices(2,1) - this.vertices(2, this.num_vertices);
            aa = ax*ax + ay*ay;
            ab = ax*bx + ay*by;
            bb = bx*bx + by*by;
            lambda = min(max(ab / bb, 0.0), 1.0);
            minSquaredDistance = aa + (bb*lambda - 2.0 * ab)*lambda;
            for k = int32(1):(this.num_vertices - int32(1))
                ax = point(1) - this.vertices(1,k);
                ay = point(2) - this.vertices(2,k);
                bx = this.vertices(1,k+1) - this.vertices(1,k);
                by = this.vertices(2,k+1) - this.vertices(2,k);
                aa = ax*ax + ay*ay;
                ab = ax*bx + ay*by;
                bb = bx*bx + by*by;
                lambda = min(max(ab / bb, 0.0), 1.0);
                minSquaredDistance = min(minSquaredDistance, aa + (bb*lambda - 2.0 * ab)*lambda);
            end
        end
        function [boundaryPoint, edge] = ClosestBoundaryPoint(this, point)
            % Calculate the closest boundary point, e.g. a point on an edge, to a given query point.
            % IMPORTANT: IT IS NOT CHECKED WHETHER THIS POLYGON IS VALID OR NOT!
            % 
            % PARAMETER
            % point ... Query point for which to calculate the closest boundary point.
            % 
            % RETURN
            % boundaryPoint ... The bounary point that is closest to the query point or a non-finite value, if this convex polygon contains less than 3 vertices.
            % edge ... 3-dimensional edge parameter vector that indicates the edge containing the boundary point. The edge is given as [a;b;c], where [a;b] is a
            %          normal vector pointing outside the polygon and the implicit line equation of that edge is f(x,y) = a*x + b*y + c = 0.
            arguments (Input)
                this ave.geometry.ConvexPolygon
                point (2,1) double
            end
            boundaryPoint = nan(2,1);
            edge = nan(3,1);
            if(this.num_vertices < int32(3))
                return;
            end
            ax = point(1) - this.vertices(1, this.num_vertices);
            ay = point(2) - this.vertices(2, this.num_vertices);
            bx = this.vertices(1,1) - this.vertices(1, this.num_vertices);
            by = this.vertices(2,1) - this.vertices(2, this.num_vertices);
            aa = ax*ax + ay*ay;
            ab = ax*bx + ay*by;
            bb = bx*bx + by*by;
            lambda = min(max(ab / bb, 0.0), 1.0);
            minDistance = sqrt(aa + (bb*lambda - 2.0 * ab)*lambda);
            boundaryPoint(1) = this.vertices(1, this.num_vertices) + lambda * bx;
            boundaryPoint(2) = this.vertices(2, this.num_vertices) + lambda * by;
            edge = this.edges(:, this.num_vertices);
            for k = int32(1):(this.num_vertices - int32(1))
                ax = point(1) - this.vertices(1,k);
                ay = point(2) - this.vertices(2,k);
                bx = this.vertices(1,k+1) - this.vertices(1,k);
                by = this.vertices(2,k+1) - this.vertices(2,k);
                aa = ax*ax + ay*ay;
                ab = ax*bx + ay*by;
                bb = bx*bx + by*by;
                lambda = min(max(ab / bb, 0.0), 1.0);
                d = sqrt(aa + (bb*lambda - 2.0 * ab)*lambda);
                if(d < minDistance)
                    minDistance = d;
                    boundaryPoint(1) = this.vertices(1,k) + lambda * bx;
                    boundaryPoint(2) = this.vertices(2,k) + lambda * by;
                    edge = this.edges(:, k);
                end
            end
        end
        function lambda = LineEdgeIntersection(this, pointA, pointB)
            % Calculate the intersection of edges with a line from pointA to pointB and select the intersection that is closest to pointA.
            % IMPORTANT: IT IS NOT CHECKED WHETHER THIS POLYGON IS VALID OR NOT!
            % 
            % PARAMETER
            % pointA ... Indicates the start point of the line.
            % pointB ... Indicates the end point of the line.
            % 
            % RETURN
            % lambda ... The intersection ratio where the line from pointA to pointB intersects with the closest edge of this polygon.
            %            This value can be any real number (positive or negative) or a non-finite value if no intersection exists.
            %            The intersection point is calculated by:
            %               pointA + lambda*(pointB - pointA)
            arguments (Input)
                this ave.geometry.ConvexPolygon
                pointA (2,1) double
                pointB (2,1) double
            end

            % edge parameters for line from A to B
            nx = pointB(2) - pointA(2);
            ny = pointA(1) - pointB(1);
            len = sqrt(nx*nx + ny*ny);
            len = 1.0 / (len + double(len == 0.0));
            line_a = len * nx;
            line_b = len * ny;
            line_c = -line_a*pointA(1) - line_b*pointA(2);

            % iterate over all edges
            lambda = NaN;
            for i = int32(0):(this.num_vertices - int32(1))
                % check if intersection exists and lies between both vertices of the edge
                v1 = this.vertices(:, i + int32(1));
                v2 = this.vertices(:, mod(i + int32(1), this.num_vertices) + int32(1));
                s = (line_a*v1(1) + line_b*v1(2) + line_c) / (line_a*(v1(1) - v2(1)) + line_b*(v1(2) - v2(2)));
                if(isfinite(s) && (s >= 0.0) && (s <= 1.0))
                    % calculate lambda ratio and check if its absolute value has been decreased
                    a = this.edges(1, i + int32(1));
                    b = this.edges(2, i + int32(1));
                    c = this.edges(3, i + int32(1));
                    lambda_i = (a*pointA(1) + b*pointA(2) + c) / (a*(pointA(1) - pointB(1)) + b*(pointA(2) - pointB(2)));
                    if(~isfinite(lambda) || (isfinite(lambda_i) && (abs(lambda_i) < abs(lambda))))
                        lambda = lambda_i;
                    end
                end
            end
        end
        function overlap = Overlap(this, polygon)
            % Check whether a convex polygon overlaps with this convex polygon.
            % IMPORTANT: IT IS NOT CHECKED WHETHER ANY POLYGON IS CONVEX OR NOT!
            % 
            % PARAMETER
            % polygon ... The convex polygon for which to check collision or overlapping.
            % 
            % RETURN
            % overlap ... True if the convex polygon overlaps with this convex polygon, false otherwise.
            arguments
                this ave.geometry.ConvexPolygon
                polygon (1,1) ave.geometry.ConvexPolygon
            end

            % check if AABB overlaps
            overlap = ~((polygon.aabb_lowerBound(1) > this.aabb_upperBound(1)) || (this.aabb_lowerBound(1) > polygon.aabb_upperBound(1)) || (polygon.aabb_lowerBound(2) > this.aabb_upperBound(2)) || (this.aabb_lowerBound(2) > polygon.aabb_upperBound(2)));
            if(~overlap)
                return;
            end
            
            % Type Edge-Vertex (edges of this with vertices of polygon input)
            inCFree = false;
            for n = int32(0):(this.num_vertices - int32(1))
                nx = this.edges(1, n + int32(1));
                ny = this.edges(2, n + int32(1));
                x1 = this.vertices(1, n + int32(1));
                y1 = this.vertices(2, n + int32(1));
                vx = polygon.vertices(1, 1) - x1;
                vy = polygon.vertices(2, 1) - y1;
                v1x = polygon.vertices(1, polygon.num_vertices) - polygon.vertices(1, 1);
                v1y = polygon.vertices(2, polygon.num_vertices) - polygon.vertices(2, 1);
                v2x = polygon.vertices(1, 2) - polygon.vertices(1, 1);
                v2y = polygon.vertices(2, 2) - polygon.vertices(2, 1);
                inCFree = inCFree || (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                if(~inCFree)
                    for k = int32(1):(polygon.num_vertices - int32(2))
                        vx = polygon.vertices(1, k + int32(1)) - x1;
                        vy = polygon.vertices(2, k + int32(1)) - y1;
                        v1x = polygon.vertices(1, k) - polygon.vertices(1, k + int32(1));
                        v1y = polygon.vertices(2, k) - polygon.vertices(2, k + int32(1));
                        v2x = polygon.vertices(1, k + int32(2)) - polygon.vertices(1, k + int32(1));
                        v2y = polygon.vertices(2, k + int32(2)) - polygon.vertices(2, k + int32(1));
                        inCFree = inCFree || (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                        if(inCFree)
                            break;
                        end
                    end
                end
                vx = polygon.vertices(1, polygon.num_vertices) - x1;
                vy = polygon.vertices(2, polygon.num_vertices) - y1;
                v1x = polygon.vertices(1, polygon.num_vertices - int32(1)) - polygon.vertices(1, polygon.num_vertices);
                v1y = polygon.vertices(2, polygon.num_vertices - int32(1)) - polygon.vertices(2, polygon.num_vertices);
                v2x = polygon.vertices(1, 1) - polygon.vertices(1, polygon.num_vertices);
                v2y = polygon.vertices(2, 1) - polygon.vertices(2, polygon.num_vertices);
                inCFree = inCFree || (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                if(inCFree)
                    break;
                end
            end

            % Type Vertex-Edge (vertices of this with edges of polygon input)
            if(~inCFree)
                for n = int32(0):(polygon.num_vertices - int32(1))
                    nx = polygon.edges(1, n + int32(1));
                    ny = polygon.edges(2, n + int32(1));
                    x1 = polygon.vertices(1, n + int32(1));
                    y1 = polygon.vertices(2, n + int32(1));
                    vx = this.vertices(1, 1) - x1;
                    vy = this.vertices(2, 1) - y1;
                    v1x = this.vertices(1, this.num_vertices) - this.vertices(1, 1);
                    v1y = this.vertices(2, this.num_vertices) - this.vertices(2, 1);
                    v2x = this.vertices(1, 2) - this.vertices(1, 1);
                    v2y = this.vertices(2, 2) - this.vertices(2, 1);
                    inCFree = inCFree || (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                    if(~inCFree)
                        for k = int32(1):(this.num_vertices - int32(2))
                            vx = this.vertices(1, k + int32(1)) - x1;
                            vy = this.vertices(2, k + int32(1)) - y1;
                            v1x = this.vertices(1, k) - this.vertices(1, k + int32(1));
                            v1y = this.vertices(2, k) - this.vertices(2, k + int32(1));
                            v2x = this.vertices(1, k + int32(2)) - this.vertices(1, k + int32(1));
                            v2y = this.vertices(2, k + int32(2)) - this.vertices(2, k + int32(1));
                            inCFree = inCFree || (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                            if(inCFree)
                                break;
                            end
                        end
                    end
                    vx = this.vertices(1, this.num_vertices) - x1;
                    vy = this.vertices(2, this.num_vertices) - y1;
                    v1x = this.vertices(1, this.num_vertices - int32(1)) - this.vertices(1, this.num_vertices);
                    v1y = this.vertices(2, this.num_vertices - int32(1)) - this.vertices(2, this.num_vertices);
                    v2x = this.vertices(1, 1) - this.vertices(1, this.num_vertices);
                    v2y = this.vertices(2, 1) - this.vertices(2, this.num_vertices);
                    inCFree = inCFree || (((nx*vx + ny*vy) > 0.0) && ((nx*v1x + ny*v1y) >= 0.0) && ((nx*v2x + ny*v2y) >= 0.0));
                    if(inCFree)
                        break;
                    end
                end
            end

            % it's a collision if no configuration is in CFree
            overlap = ~inCFree;
        end
        function hull = CreateTransformedConvexHull(this, x0, y0, cosPsi0, sinPsi0, x1, y1, cosPsi1, sinPsi1)
            % Create a convex hull polygon from all vertices of two transformed polygons.
            % 
            % PARAMETER
            % x0 ... Translation in the x direction for configuration 0.
            % y0 ... Translation in the y direction for configuration 0.
            % cosPsi0 ... The cosine of rotation angle psi for configuration 0.
            % sinPsi0 ... The sine of rotation angle psi for configuration 0.
            % x1 ... Translation in the x direction for configuration 1.
            % y1 ... Translation in the y direction for configuration 1.
            % cosPsi1 ... The cosine of rotation angle psi for configuration 1.
            % sinPsi1 ... The sine of rotation angle psi for configuration 1.
            % 
            % RETURN
            % hull ... A convex polygon that indicates the convex hull of two transformed polygons.
            arguments
                this ave.geometry.ConvexPolygon
                x0 (1,1) double
                y0 (1,1) double 
                cosPsi0 (1,1) double
                sinPsi0 (1,1) double
                x1 (1,1) double
                y1 (1,1) double
                cosPsi1 (1,1) double
                sinPsi1 (1,1) double
            end

            % transform vertices by two configurations and create point cloud
            points = zeros(2, this.num_vertices * int32(2));
            for k = int32(1):this.num_vertices
                points(1, k) = x0 + cosPsi0 * this.vertices(1, k) - sinPsi0 * this.vertices(2, k);
                points(2, k) = y0 + sinPsi0 * this.vertices(1, k) + cosPsi0 * this.vertices(2, k);
                points(1, k + this.num_vertices) = x1 + cosPsi1 * this.vertices(1, k) - sinPsi1 * this.vertices(2, k);
                points(2, k + this.num_vertices) = y1 + sinPsi1 * this.vertices(1, k) + cosPsi1 * this.vertices(2, k);
            end

            % compute the convex hull
            ihull = ave.geometry.QuickHull2D(points);
            hull = ave.geometry.ConvexPolygon(this.num_vertices * int32(2));
            hull.Create(points(:,ihull));
        end
        function vertices = GetVertices(this)
            % Get vertices of this convex polygon as a variable-size matrix.
            % 
            % RETURN
            % vertices ... Section copy of the internally stored vertices, given as 2-by-N matrix with N being the number of vertices.
            arguments (Input)
                this ave.geometry.ConvexPolygon
            end
            vertices = this.vertices(:, int32(1):this.num_vertices);
        end
        function [vertices, num_vertices] = GetVerticesStorage(this)
            % Get the constant-size vertices storage of this convex polygon.
            % 
            % RETURN
            % vertices ... Copy of the internally stored vertices, given as 2-by-N matrix with N being the number of vertices specified during construction.
            % num_vertices ... The actual number of vertices that indicate the convex polygon.
            arguments (Input)
                this ave.geometry.ConvexPolygon
            end
            vertices = this.vertices;
            num_vertices = this.num_vertices;
        end
        function plot(this, varargin)
            % Plot this convex polygon as a polyshape.
            assert(isa(this, 'ave.geometry.ConvexPolygon'));
            plot(polyshape(this.vertices(:, int32(1):this.num_vertices)'), varargin{:});
        end
    end
    methods(Access = protected)
        function RebuildEdge(this, idx, idxNext)
            nx = this.vertices(2, idxNext) - this.vertices(2, idx);
            ny = this.vertices(1, idx) - this.vertices(1, idxNext);
            len = sqrt(nx*nx + ny*ny);
            len = 1.0 / (len + double(len == 0.0));
            this.edges(1, idx) = len * nx;
            this.edges(2, idx) = len * ny;
            this.edges(3, idx) = -this.edges(1,idx) * this.vertices(1,idx) - this.edges(2,idx) * this.vertices(2,idx);
        end
    end
    properties(Access = protected)
        num_vertices (1,1) int32
        vertices (2,:) double
        edges (3,:) double % edge(:,i) goes from vertices(:,i) to vertices(:,i+1)
        aabb_lowerBound (2,1) double
        aabb_upperBound (2,1) double
    end
end
