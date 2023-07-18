function val = intBernstein(poly,int)
%INTBERNSTEIN integrates the poly over its interval
val = sum(poly) *( (int(end)-int(start))/ ...
                   (size(poly,2))-1+1 );
end

