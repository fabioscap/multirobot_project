function problem = planningProblem(Env)
%PLANNINGPROBLEM Builds the simplified planning problem using
% Bernstein approximants
    
% we want to search for:
% tf: final absolute time
% pd: bernstein coefficients for the trajectories of the agents 
%     for the interval [Env.t, tf]
%     shape: (dimxN_degreexN_agents)

end

function y = costFunction(x, Env)
    w1 = 1.0;
    w2 = 1.0;
    w3 = 1.0;

    % get the quantities of interest from x
    tf = x(1);
    p = reshape(x(2:end), Env.dim, [], length(Env.agents));

    y = w1 * minTime(tf, Env) + ...
        w2 * minEffort(tf, p, Env) + ...
       -w3 * maxObservability(tf, p, Env);
end

function y = constraintsNL(x, Env) 
    % get the quantities of interest from x
    tf = x(1);
    p = reshape(x(2:end), Env.dim, [], length(Env.agents));
    
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
    y = obsIndex(p,[Env.t, tf], Env.tau);

end


% define the constraints

% 15a 
% starting position (no velocity for now)
function y = startingPosition(p)
    y = squeeze(p(:,1,:));
end

% 15b
% get to the estimated target
function y = endingPosition(p, Env)
    p_f = squeeze(p(:,end,:)); % should be dimxA
    p_t = reshape(Env.p_hat, [], 1);
    deltas = p_f - p_t;
    y = vecnorm(deltas, 2, 1); % should be 1xA;
end

% 15c interdistance TODO

% 15d safe velocity TODO
