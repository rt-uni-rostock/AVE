function convexPolygons = ConvexPolyshapePartitioning(polygon)
    %ave.ConvexPolyshapePartitioning Make a convex partitioning for a given 2D polyshape.
    % 
    % INPUT
    % polygon ... [1 x polyshape] A polyshape representing a possibly non-convex polygon that is to be partitioned into several convex polygons.
    % 
    % RETURN
    % convexPolygons ... [N x polyshape] List of polyshapes representing the set of convex polygons.
    % 
    % DETAILS
    % The partitioning is not guaranteed to be optimal. The algorithm is not optimized and is designed to run in MATLAB.
    % It provides a simple procedure to calculate a convex partition for a given (non-convex) polygon. First, the input
    % polygon is tesselated into a set of triangles using MATLABs delauny-triangulation. Those triangles are sorted in
    % descending order depending on their area. Finally, all neighbor triangles (or convex polygons) are tried to combine
    % into a single convex polygon in a recursive manner.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221223    Robert Damerius        Initial release.
    % 20251027    Robert Damerius        Process regions separately to increase performance. Check regions and holes when
    %                                    merging two triangles.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(isa(polygon, "polyshape"), "Input ""polygon"" must be of type polyshape!");
    assert(isscalar(polygon), "Input ""polygon"" must contain only one polyshape!");

    % Default output
    convexPolygons = polyshape.empty();

    % Process region by region
    regions = polygon.regions();
    for i = 1:numel(regions)
        result = RegionPartitioning(regions(i));
        if(isempty(convexPolygons))
            convexPolygons = result;
        else
            convexPolygons = [convexPolygons, result];
        end
    end
end


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Private Helper Functions
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function convexPolygons = RegionPartitioning(polygon)
    convexPolygons = polyshape.empty();
    if(isempty(polygon.Vertices) || (polygon.area() <= eps))
        return;
    end

    % Tesselate into triangles and sort them by area (starting with largest one)
    triangles = Triangulate(polygon);
    convexPolygons = SortByArea(triangles);

    % Try to fuse neighbor triangles and combine them into one convex polygon (via union)
    i = 1;
    while(i <= numel(convexPolygons))
        jFusable = -1;
        for j = (i+1):numel(convexPolygons)
            % Two polygons are fusable, if their union has the same area as the area of their convex hull
            if(Fusable(convexPolygons{i}, convexPolygons{j}))
                jFusable = j;
                break;
            end
        end
        if(jFusable > 0)
            convexPolygons{i} = union(convexPolygons{i}, convexPolygons{jFusable});
            convexPolygons(jFusable) = [];
            continue;
        end
        i = i + 1;
    end
end

function triangles = Triangulate(polygon)
    DT = triangulation(polygon);
    triangles = cell.empty();
    for i = 1:size(DT.ConnectivityList,1)
        verticesTriangles = DT.Points(DT.ConnectivityList(i,:)',:);
        triangles{i} = polyshape(verticesTriangles);
    end
end

function polygonsOut = SortByArea(polygonsIn)
    area = zeros(numel(polygonsIn),1);
    for i = 1:numel(polygonsIn)
        area(i) = polygonsIn{i}.area();
    end
    [~,idx] = sort(area,'descend');
    polygonsOut = polygonsIn(idx);
end

function success = Fusable(convexPolygon1, convexPolygon2)
    success = false;
    vertices = [convexPolygon1.Vertices; convexPolygon2.Vertices];
    if(isempty(vertices)), return; end
    i = convhull(vertices);
    if(numel(i) < 3), return; end
    convexHullPolygon = polyshape(vertices(i,:));
    unionPolygon = union(convexPolygon1, convexPolygon2);
    success = (1 == unionPolygon.NumRegions) && (0 == unionPolygon.NumHoles) && (abs(convexHullPolygon.area() - unionPolygon.area()) <= eps);
end
