function A = laplacian2adj(L)
    % the graph we use are undirected and do not have loops
    % we can just remove the diagonal elements and negate it
    A = eye(size(L)).*diag(L) - L;
end

