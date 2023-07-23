function extr = extrBernstein(poly, int, alpha, tol)
%EXTRBERNSTEIN finds the extrema of a 1D bernstein polynomial
% Alg. 1 of Bernstein polynomial-based transcription method for solving optimal
% trajectory generation problems
arguments
    poly
    int
    alpha = 0 % lower bound for maximun
    tol = 1e-3
end
    %pause()
    N = size(poly,2)-1;
    dim = size(poly,1);
    if dim ~= 1
        error("only 1D polynomials are supported");
    end
    
    % find the lower bound by comparing end points
    lb = max(poly(1), poly(end));
    % find the upper bound as the largest point
    [ub, i_ub] = max(poly);
    if lb > alpha
        alpha = lb;
    end
    if alpha >= ub
        extr = alpha;
        return
    end
    if ub - lb < tol
        extr = lb;
        return
    end
    % disp("alpha:" + alpha);
    % disp("lower bound: " + lb);
    % disp("upper bound:" + ub);
    % split
    t_split = ((i_ub-1)/N)*(int(end)-int(1)) + int(1);
    
    % not in the paper but necessary for convergence
    if t_split == int(end) || t_split == int(1)
        extr = alpha;
        return
    end
    [~, pa, pb] = deCasteljau(poly, t_split, int);
    % disp("calling on left split: " + int(1) + ":" + t_split )
   
    extr_a = extrBernstein(pa, [int(1), t_split], alpha);
    % disp("calling on right split: " + t_split + " :" + int(end));
 
    extr_b = extrBernstein(pb, [t_split, int(end)], alpha);
    
    extr = max(extr_a, extr_b);
end