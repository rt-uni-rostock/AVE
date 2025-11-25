function [pgon, ngon] = VertexListToConvexPolygons(vertices)
    %VertexListToConvexPolygons Convert a list of vertices to convex polygons, where polygons are separated by non-finite vertices.
    % 
    % PARAMETER
    % vertices ... 2-by-N matrix of vertices of convex polygons. Polygons are seprated by non-finite vertices.
    % 
    % RETURN
    % pgon ... Constant-size cell array of convex polygons.
    % ngon ... Actual number of converted polygons in pgon.
    arguments (Input)
        vertices (2,:) double
    end
    N = int32(size(vertices,2));
    max_polygons = ceil(double(N) / 2.0);

    % define output (required for code generation)
    pgon = cell(max_polygons,1);
    for i = 1:max_polygons
        pgon{i} = ave.geometry.ConvexPolygon(N);
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
