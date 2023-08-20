function expandedMatrix = expandLaplacian(L, dim)
    % Check if the input matrix is square
    [m, n] = size(L);
    if m ~= n
        error('Input matrix must be square (Laplacian matrix)');
    end
    
    % Check if the dim is valid
    if dim < 1
        error('Dimension must be a positive integer');
    end
    
    % Initialize the expanded matrix
    expandedMatrix = zeros(n * dim, n * dim);
    
    % Fill in the expanded matrix
    rowStart = 1;
    for i = 1:n
        colStart = 1;
        for j = 1:n
            for k=0:dim-1
                expandedMatrix(rowStart+k,colStart+k) = L(i,j);
            end
            colStart = colStart + dim;
        end
        rowStart = rowStart+dim;
    end
end