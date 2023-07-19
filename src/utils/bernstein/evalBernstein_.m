function value = evalBernstein_(poly, t, int)
%EVALBERNSTEIN Evaluate Bernstein polynomial at query point t
% the bernstein polynomial is defined in interval [int(start),int(end)]

% do not use this use deCasteljau

t0 = int(1);
tf = int(2);

N = size(poly,2)-1;

value = zeros(size(poly,1),1);
for i=0:N
    % eq. 10 Optimal Motion Planning for Localization ...
    value = value + poly(:,i+1)*nchoosek(N,i)*(t-t0)^i * (tf -t)^(N-i);           
end

value = value / (tf-t0)^N;

end