% builds the phi vector from a 3d or 2d position
function phi = buildPhi(p)
    if length(p) == 3
    phi = [p(1)^2; 2*p(1)*p(2); 2*p(1)*p(3);...
           p(2)^2; 2*p(2)*p(3); p(3)^2;     ...
          -2*p(1);-2*p(2);-2*p(3);  1];
    else
    phi = [p(1)^2; 2*p(1)*p(2); p(2)^2; -2*p(1); -2*p(2); 1];
    end
end