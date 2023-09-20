function A = starConnection(N)
%STARCONNECTION returns an adjacency matrix where the nodes are connected
% in a circle
    A = zeros(N,N);
    A(2:N, 1) = 1;
    A(1, 2:N) = 1;
end