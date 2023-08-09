function x_dot = doubleInt(t, x, u)
    %DOUBLEINT double integrator dynamics
    x_dot = zeros(length(x),1);
    
    middle = length(x) /2;
    
    x_dot(1:middle) = x(middle+1:end);
    x_dot(middle+1:end) = u;
end