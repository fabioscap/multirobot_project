% inverse mapping to get p_hat back from x
function target = extractTarget(x, a, b)
    if length(x) == 10
        % x is (m11, m12, m13, m22, m23, m33, p1, p2, p3, d)
        M = [x(1) x(2) x(3);
             x(2) x(4) x(5);
             x(3) x(5) x(6)];
        y = [x(7);x(8);x(9)];
    elseif length(x) == 6 
        % x is (m11, m12, m22, p1, p2, d)
        M = [x(1) x(2);
             x(2) x(3)];
        y = [x(4); x(5)];
    else
        error("unknown x dimension " + length(x));
    end
    % clamp the singular values to have a
    % consistent estimate (paper 1)

    % this makes 
    %M = saturateSValues(M, a^2, b^2);

    target = M\y;
end

function M_bar = saturateSValues(M,s1,s2,k)
    arguments
       M
       s1
       s2
       k=1.0 % tolerance factor >=1(?)
    end
    [U, S, V] = svd(M);
    ub = k * max(s1, s2);
    lb = min(s1, s2) / k;
    S_bar = zeros(size(S,1), size(S,2));

    for i=1:length(diag(S))
        S_bar(i,i) = clamp(S(i,i), lb, ub);
    end

    M_bar = U*S_bar*V';

end


