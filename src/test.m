%clear all; close all; clc; addpath(genpath(pwd));

% the actual target pose (unknown to the agents)
% [ x; y; theta];
p_t = [ 0; 0; 0];


% the number of agents
n_agents = 3;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.1;

e = Environment(2, n_agents, p_t, tau);


% TODO define how to handle
% orientations: do we need them?
% 2D/3D: adapth the observability index in 2D and the whole estimation
% pipeline

[tf, p] = planningProblem(e);

f = 1;
int = [0,tf];

for a=1:length(e.agents)
    plotBernstein(p(:,:,a), int, f); hold on;

end



