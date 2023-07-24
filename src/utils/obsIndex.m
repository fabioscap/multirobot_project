function sigma = obsIndex(poly, int, tau)
%OBSINDEX get observability index from a bernstein polynomial trajectory,
% a reference time interval and a sampling time
% poly has shape (dim, N, agents)
dim = size(poly,1);
n_agents = size(poly,3);
s_tf = floor(int(end)/tau);
s_t0 = floor(int(1)/tau);

if dim == 2
    dim_o = 6;
elseif dim == 3
    dim_o = 10;
end

O = zeros(dim_o, dim_o);
% eq. 7
for i=s_t0+1:s_tf
    % tau_i: the sample time of the ith sample
    tau_i = i*tau;
    H = zeros(dim_o, n_agents);
    for a=1:n_agents
        % the bernstein poly. of agent a
        p_a = poly(:,:,a);
        % where agent a will be at time tau_i
        pos = deCasteljau(p_a, tau_i, int);
        % build phi vector
        H(:,a) = buildPhi(pos);
    end
    O = O + H*H';
end

O = O / (s_tf - s_t0);
svalues = svd(O);
sigma = min(svalues);

end