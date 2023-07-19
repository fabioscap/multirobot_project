function [val, split1, split2] = deCasteljau(poly, t, int)
%DECASTELJAU 

window = int(end)-int(1);
N = size(poly,2)-1;
dim = size(poly,1);
P = zeros(dim, N+1, N+1);
P(:,:,1) = poly;

for j=1:N
    for i=0:N-j
        P(:,i+1,j+1) = ((int(end)-t)*P(:,i+1,j) + (t-int(1))*P(:,i+2,j)) / window;
    end
end


val = P(:,1,N+1);
split1 = reshape(P(:,1,:),dim, N+1);

%tmp = (reshape(flip(P,3), 1,(N+1)*(N+1), []));
%split2 = reshape(tmp(1,1:N+2:end,:), dim, N+1);
split2 = zeros(dim, N+1);
for k=0:N
    split2(:,k+1) = P(:,k+1,N-k+1);
end

end