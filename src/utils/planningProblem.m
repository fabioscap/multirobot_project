function problem = planningProblem(Env)
%PLANNINGPROBLEM Builds the simplified planning problem using
% Bernstein approximants
    
% we want to search for:
% tf: final absolute time
% pd: bernstein coefficients for the trajectories of the agents 
%     for the interval [Env.t, tf]
%     shape: (dimxN_degreexN_agents)

end

% define the individual cost functions terms

% minimize mission time
% TODO try to search for an interval t instead of tf
% so that tf = Env.t + t (probably works better)
function y = minTime(tf, Env)
    y = tf - Env.t;
end

% minimize actuation effort
% TODO change the problem so you actaully search
% for p_ddot directly instead and integrate the coefficients
% later
function y = minEffort(tf, p, Env)
    int = tf - Env.t;
    y = 0;
    for a=1:length(Env.agents)
        p_ddot = diffBernstein(diffBernstein(p(:,:,a),int), int);
        nrm    = sqNormBernstein(p_ddot);
        effort = intBernstein(nrm, int);

        y = y+effort;
    end
end

% maximize the observability performance index
function y = maxObservability(tf, p, Env)
    s_tf = floor(tf/Env.tau);
    s_t0 = floor(Env.t/Env.tau);

    O = zeros(10,10);
    % eq. 7
    for i=s_t0+1:s_tf
        % tau_i: the sample time of the ith sample
        tau_i = i*Env.tau;
        H = zeros(10, obj.N);
        for a=1:length(Env.agents)
            % the bernstein poly. of agent a
            p_a = p(:,:,a);
            % where agent a will be at time tau_i
            pos = evalBernstein(p_a, tau_i, [tf, Env.t]);
            if length(pos) == 2
                pos = [p; 0];
            end
            % build phi vector
            H(:,a) = [pos(1)^2; 2*pos(1)*pos(2); 2*pos(1)*pos(3);...
                      pos(2)^2; 2*pos(2)*pos(3); pos(3)^2;     ...
                     -2*pos(1);-2*pos(2);-2*pos(3);  1];
        end
        O = O + H*H';
    end

    O = O / (s_tf - s_t0);

    y = min(svd(O));

end