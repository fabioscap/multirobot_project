% test for trajectory observability index

% it would be nice to do a plot comparing n_agents vs observability
% and trajectory shape vs observability like in the second paper but
% with more examples and a graph
int = [0,1];
tau = 0.1;


syms t
assume(t,"real");

% 8 shape
x = 0.5*sin(2*pi*t);
y = 0.5*sin(4*pi*t);

% ellipses
%x = 0.5*sin(2*pi*t);
%y = 0.2*cos(2*pi*t);

P = [x+3 y-3; x-3 y+3; x+3 y+3; x-3 y-3]';
n_agents = size(P,2);

for a=1:n_agents
    fplot(P(1,a), P(2,a)); hold on; 
end


axis equal

s_t0 = 0;
s_tf = floor(int(end)/tau);

O = zeros(6, 6);
% eq. 7
for i=s_t0+1:s_tf
    % tau_i: the sample time of the ith sample
    tau_i = i*tau;
    H = zeros(6, n_agents);
    for a=1:n_agents
        % where agent a will be at time tau_i
        pos = subs(P(:,a), tau_i);
        % build phi vector
        H(:,a) = buildPhi(pos);
    end
    O = O + H*H';
end

O = O / (s_tf - s_t0);
svalues = svd(O);
disp("Observability index: ");
sigma = min(svalues)

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

