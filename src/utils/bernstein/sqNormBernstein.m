function out = sqNormBernstein(poly)
%SQNORMBERNSTEIN squared norm of a bernstein polynomial 
% (it is a bernstein polynomial)

% compute pk*pk for k=1,...,D for each dimension
% using the formula for the product of 1D Bernstein Polynomials
% (The Bernstein polynomial basis: a centennial retrospective)
% then sum the results

% compute the dimension
D = size(poly,1);
% compute the order
N = size(poly,2)-1;

% store the intermediate products
prod = zeros(size(poly,1),2*N+1);

% TODO three for loops...
for i=0:2*N
    den = nchoosek(2*N, i);
    for j=max(0,i-N):min(N,i)
        factor = nchoosek(N,j)*nchoosek(N,i-j)/den ;
        for d=1:D
            prod(d,i+1) = prod(d,i+1) + factor * poly(d,j+1) * poly(d, i-j +1);
        end
    end 
end

% the sum of two polynomials is the sum of the coefficients
out = sum(prod, 1);
end

