clear all; close all; clc; addpath(genpath(pwd));

dim = 2;

% the actual target pose (unknown to the agents)
% [ x; y; theta];
p_t = zeros(dim, 1);


% the number of agents
n_agents = 4;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.1;

e = Environment(dim, n_agents, p_t, tau);
%e.sim();


% TODO RLS diverges
% TODO add noise !!!!!

[int, p] = planningProblem(e);

f = 1;

for a=1:e.n_agents
    plotBernstein(p(:,:,a), int, f); hold on;
end



