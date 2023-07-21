function [int , p] = planningProblem(Env)
%PLANNINGPROBLEM Builds the simplified planning problem using
% Bernstein approximants
    
% we want to search for:
% tf: final absolute time
% pd: bernstein coefficients for the trajectories of the agents 
%     for the interval [Env.t, tf]
%     shape: (dimxorderxN_agents)
    order = 5; % fifth order bernstein approx.
    
    x0 = [Env.t+ 0.1;zeros(Env.dim* order* length(Env.agents),1)];

    lb = -inf*ones(size(x0));
    lb(1) = Env.t + 0.001 ; % we want tf to be bigger than the current time

    cf = @(x) costFunction(x, Env);


    nlc = @(x) constraintsNL(x, Env);
    options = optimoptions(@fmincon);
    % TODO adjust this number
    options = optimoptions(@fmincon,'MaxFunctionEvaluations',200);
    % TODO see if we can add early stopping
    [x, ~] = fmincon(cf, x0, [], [], [], [], lb, [], nlc, options);

    % get the quantities of interest from x
    tf = x(1);
    int = [Env.t, tf];
    p = reshape(x(2:end), Env.dim, order, length(Env.agents));
    int
end

function y = costFunction(x, Env)
    w1 = 0.3;
    w2 = 1;
    w3 = 0.01;

    % get the quantities of interest from x
    tf = x(1);
    p = reshape(x(2:end), Env.dim, [], length(Env.agents));
    
    y = w1 * minTime(tf, Env) + ...
        w2 * minEffort(tf, p, Env) + ...
       -w3 * maxObservability(tf, p, Env);
end

% let's try to put every constraint here (even the linear ones)
% and if it does not work  split them
function [c_in, c_eq] = constraintsNL(x, Env) 
    % get the quantities of interest from x

    % c_in <= 0
    c_in = [];
    % c_eq == 0
    c_eq = [];

    tf = x(1);
    p = reshape(x(2:end), Env.dim, [], length(Env.agents));
    
    % arrive near estimated target
    delta = 3.0;
    c_in = [ c_in ; endingPosition(p, Env) - delta];

    % inter vehicle distance TODO

    % starting positions
    positions = zeros(Env.dim, length(Env.agents));
    c0        = reshape(p(:,1,:), [],1);
    for a=1:length(Env.agents)
        positions(:,a) = Env.agents(a).position;
    end
    positions = reshape(positions,[], 1);
    c_eq = [c_eq; c0 - positions];
  
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
    int = [Env.t, tf];
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
function y = maxVelocity(tf, p, Env)
    for a=1:length(Env.agents)
        int = [Env.t, tf];
        p_a = p(:,:,a);
        pd_a = diffBernstein(p_a, int);

        speed_a = sqNormBernstein(pd_a);

    end

end
