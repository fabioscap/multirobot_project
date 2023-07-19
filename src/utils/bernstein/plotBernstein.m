function f = plotBernstein(poly, f)
arguments
    poly
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

samples = linspace(0, 1, n_samples);

for i=1:length(samples)
    values(:,i) = deCasteljau(poly, samples(i), [0,1]);
end
for i=1:size(poly,2)
    scatter(poly(1,i),poly(2,i), "red", "o");
end
plot(poly(1,:),poly(2,:), "red")


plot(values(1,:),values(2,:));
axis equal
end
