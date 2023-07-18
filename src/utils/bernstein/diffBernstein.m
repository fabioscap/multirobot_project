function poly_dot = diffBernstein(poly,int)
%DIFFBERNSTEIN gets the bernstein coefficients of the differentiated poly
window = int(end)-int(start);

N = size(poly,2)-1;

const = N / window;

poly_dot = diff(poly,1,2)*const;
end

