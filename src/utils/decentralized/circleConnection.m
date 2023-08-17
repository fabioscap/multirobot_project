function A = circleConnection(N)
%CIRCLECONNECTION returns an adjacency matrix where the nodes are connected
% in a circle
    A = zeros(N,N);
    for i=1:N
        prev = mod((i-2),N) + 1;
        succ = mod((i),N)   + 1;
        A(i,prev) = 1;
        A(i,succ) = 1;
    end
end

