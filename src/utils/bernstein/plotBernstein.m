function f = plotBernstein(poly, int, f, col)
arguments
    poly
    int=[0,1]
    f=-1
    col = "red"
end

n_samples = 100;
if f == -1
   f =  figure();
else
    figure(f)
end
hold on;
dim = size(poly,1);
values = zeros(dim,n_samples);

if dim == 2
    % plot x over y
    samples = linspace(int(1), int(end), n_samples);
    
    for i=1:length(samples)
        values(:,i) = deCasteljau(poly, samples(i), int);
    end

    % plot control points TODO add enable/disabler for this option
    % for i=1:size(poly,2)
    %     scatter(poly(1,i),poly(2,i), col, "o");
    % end
    % plot(poly(1,:),poly(2,:), col)
    
    
    plot(values(1,:),values(2,:), col);
    axis equal

elseif dim == 1
    % plot poly over time
    t = linspace(int(1),int(end) ,n_samples);

    for i=1:length(t)
        values(:,i) = deCasteljau(poly, t(i), int);
    end
    plot(t, values, col);
end
