function val = intBernstein(poly,int)
%INTBERNSTEIN integrates the poly over its interval
val = sum(poly) *( (int(end)-int(1))/ ...
                   (size(poly,2))-1+1 );
end

