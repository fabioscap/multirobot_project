function f = plotBernstein(poly, int, f)
arguments
    poly
    int
    f=-1
end

n_samples = 100;
if f == -1
   f =  figure();
else
    figure(f)
end
hold on;
dim = size(poly,1);
values = zeros(2,n_samples);

if dim ~= 2
    error("use 2D polynomials");
end

samples = linspace(int(1), int(end), n_samples);

for i=1:length(samples)
    values(:,i) = evalBernstein_(poly, samples(i), int);
end
for i=1:size(poly,2)
    scatter(poly(1,i),poly(2,i), "red", "o");
end


plot(values(1,:),values(2,:));
axis equal
end
