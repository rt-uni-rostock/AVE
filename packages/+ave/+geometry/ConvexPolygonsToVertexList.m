function vertices = ConvexPolygonsToVertexList(pgon, ngon, numOutputVertices)
    %ConvexPolygonsToVertexList Convert a cell array of convex polygons to a list of vertices, where polygons are separated by non-finite vertices.
    % 
    % PARAMETER
    % pgon ... Constant-size cell array of convex polygons.
    % ngon ... Number of polygons in pgon to be converted.
    % numOutputVertices ... Sets size of the output matrix.
    % 
    % RETURN
    % vertices ... 2-by-N matrix of vertices of convex polygons. Polygons are seprated by non-finite vertices. N is equal to numOutputVertices.
    arguments (Input)
        pgon cell
        ngon (1,1) int32
        numOutputVertices (1,1) uint32
    end
    arguments (Output)
        vertices (2,:) double
    end
    vertices = nan(2, numOutputVertices);
    vertex_offset = int32(1);
    for i = int32(1):ngon
        if(vertex_offset > int32(numOutputVertices))
            break;
        end
        [pvert, nvert] = pgon{i}.GetVerticesStorage();
        numCopy = min(nvert, int32(numOutputVertices) - vertex_offset + int32(1));
        vertices(:, vertex_offset:(vertex_offset + numCopy - int32(1))) = pvert(:, 1:numCopy);
        vertex_offset = vertex_offset + numCopy;
        if(vertex_offset <= int32(numOutputVertices))
            vertices(:,vertex_offset) = nan(2, 1);
            vertex_offset = vertex_offset + int32(1);
        end
    end
end
