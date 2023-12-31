function [int_final , p_final] = planningProblem(startingPos, t, targetEst, tau)
%PLANNINGPROBLEM Builds the simplified planning problem using
% Bernstein approximants

% Bernstein settings
order = 4;
dim = size(startingPos, 1);
startingPos = reshape(startingPos, dim , 1, []);
n_agents = size(startingPos, 3);

% starting time       t
% starting positions  startingPos
% target estimate     targetEst
% ARTVA sampling time tau

% we want to search for:
% T : trajectory duration
% pd: bernstein coefficients for the trajectories of the agents 
%     for the interval [Env.t, Env.t + T]
%     shape: (dimxorderxN_agents)
%     the first coefficient is hardcoded to the current positions of
%     the agents
    x0 = [10;reshape(repmat(startingPos,1,order,1), [],1)];

    lb = -100*ones(size(x0));
    ub = 100*ones(size(x0));
    lb(1) = 0;
    ub(1) = 50;

    cf = @(x) costFunction(x);
    nlc = @(x) constraintsNL(x);

    options = optimoptions(@fmincon);
    % TODO adjust this number 
    % performance highly depends on x0 and the weights
    % find a setting that is accurate and does not take ages
    options = optimoptions(@fmincon,"MaxFunctionEvaluations", 200);
    % TODO see if we can add early stopping
    [x, ~] = fmincon(cf, x0, [], [], [], [], lb, ub, nlc, options);
    
    % DEBUG visualization: check the constraints values
    [c_in, c_eq] = constraintsNL(x)

    % get the quantities of interest from x
    int_final = [t, t + x(1)];
    p_final = cat(2,startingPos,reshape(x(2:end), dim, [], n_agents));


function y = costFunction(x)
    w1 = 0.9;
    w2 = 1;
    w3 = 0.3;

    % get the quantities of interest from x
    T = x(1);
    p = cat(2,startingPos,reshape(x(2:end), dim, [], n_agents));
    
    y = w1 * minTime(T) + ...
        w2 * minEffort(T, p) + ...
       -w3 * maxObservability(T, p);
end

% let's try to put every constraint here (even the linear ones)
% and if it does not work  split them
function [c_in, c_eq] = constraintsNL(x) 
    % c_in <= 0
    c_in = [];
    % c_eq == 0
    c_eq = [];
    
    % get the quantities of interest from x
    T = x(1);
    p = cat(2,startingPos,reshape(x(2:end), dim, [], n_agents));
    
    % arrive near estimated target
    delta = 1.0;
    c_in = [ c_in ; endingPosition(p) - delta];

    % inter vehicle distance TODO

    % safe velocity (actually the square)
    speed_thr = 100; % = 7 m/s * 7 m/s
    max_speed = maxSpeed(T, p);
    c_in = [c_in; max_speed - speed_thr];
  
end

% define the individual cost functions terms

% minimize mission time
function y = minTime(T)
    y = T;
end

% minimize actuation effort
% TODO change the problem so you actaully search
% for p_ddot directly instead and integrate the coefficients
% later
function y = minEffort(T, p)
    int = [t, t+T];
    y = 0;
    for a=1:n_agents
        p_ddot = diffBernstein(diffBernstein(p(:,:,a),int), int);
        nrm    = sqNormBernstein(p_ddot);
        effort = intBernstein(nrm, int);

        y = y+effort;
    end
end

% maximize the observability performance index
function y = maxObservability(T, p)
    y = obsIndex(p,[t, t+T], tau);

end


% define the constraints

% 15a 
% starting position (no velocity for now)
function y = startingPosition(p)
    y = squeeze(p(:,1,:));
end

% 15b
% get to the estimated target
function y = endingPosition(p)
    p_f = squeeze(p(:,end,:)); % should be dimxA
    p_t = reshape(targetEst, [], 1);
    deltas = p_f - p_t;
    y = reshape(vecnorm(deltas, 2, 1),[],1); % should be Ax1;
end

% 15c interdistance TODO

% 15d safe velocity TODO
function y = maxSpeed(T, p)
    y = zeros(n_agents, 1);
    int = [t, t + T];
    for a=1:n_agents
        p_a = p(:,:,a);
        pd_a = diffBernstein(p_a, int);

        speed_a = sqNormBernstein(pd_a);
        y(a) = extrBernstein(speed_a, int);
    end

end

function stop = customOutputFcn(x, optimValues, state)
    % Check for feasibility of the current solution
    if any(optimValues.constrviolation <= 0)
        stop = true; % Terminate the optimization if any constraint is satisfied
    else
        stop = false; % Continue optimization if no constraint is satisfied
    end
end

end