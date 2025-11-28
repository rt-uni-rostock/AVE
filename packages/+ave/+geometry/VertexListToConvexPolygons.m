function [pgon, ngon] = VertexListToConvexPolygons(vertices, maxNumPolygons, maxVerticesPerPolygon)
    %VertexListToConvexPolygons Convert a list of vertices to convex polygons, where polygons are separated by non-finite vertices.
    % 
    % PARAMETER
    % vertices ... 2-by-N matrix of vertices of convex polygons. Polygons are seprated by non-finite vertices.
    % maxNumPolygons ... (Optional) Set the maximum number of polygons to be stored in a cell array. This value is limited to be at most 1024. Set to zero to automatically calculate. Default value is 0.
    % maxVerticesPerPolygon ... (Optional) Set the maximum number of vertices to be stored for a convex polygon. This value is limited to be at most N. Set to zero to automatically calculate. Default value is 0.
    % 
    % RETURN
    % pgon ... Constant-size cell array of convex polygons.
    % ngon ... Actual number of converted polygons in pgon.
    arguments (Input)
        vertices (2,:) double
        maxNumPolygons (1,1) uint32 = 0
        maxVerticesPerPolygon (1,1) uint32 = 0
    end
    N = int32(size(vertices,2));

    % set cell array size
    max_polygons = uint32(ceil(double(N + int32(1)) / 4)); % automatic size selection
    if(maxNumPolygons)
        max_polygons = maxNumPolygons; % user-specific size selection
    end
    max_polygons = min(max_polygons, uint32(1024)); % code generation requires cell arrays to contain at most 1024 elements

    % set vertex storage size
    max_vertices = N;
    if(maxVerticesPerPolygon)
        max_vertices = int32(min(maxVerticesPerPolygon, uint32(N)));
    end

    % define output (required for code generation)
    pgon = cell(max_polygons,1);
    for i = 1:max_polygons
        pgon{i} = ave.geometry.ConvexPolygon(max_vertices);
    end
    ngon = int32(0);

    % go through vertex list
    i0 = int32(-1);
    for i = int32(1):N
        finiteVertex = isfinite(vertices(1,i)) && isfinite(vertices(2,i));
        if((i0 < 1) && finiteVertex)
            i0 = i;
        elseif((i0 > 0) && (~finiteVertex || (i == N)))
            ie = (i + int32((i == N) && finiteVertex)) - int32(1);
            ngon = ngon + int32(1);
            pgon{ngon}.Create(vertices(:, i0:ie));
            i0 = int32(-1);
        end
    end
end
