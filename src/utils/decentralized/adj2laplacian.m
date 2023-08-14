function L = adj2Laplacian(A)
    D = eye(size(A)).*sum(A, 1);

    L = D - A;
end

