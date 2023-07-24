clear all; close all; clc; addpath(genpath(pwd));

dim =2 ;

% the actual target pose (unknown to the agents)
% [ x; y];
p_t = zeros(dim, 1);

p_t = [10;20];

% the number of agents
n_agents = 4;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.5;

e = Environment(dim, n_agents, p_t, tau);
e.sim();


% RLS now converges
% I had to put a different formulation (chatGPT) and 
% removed clamping from extractTarget even if the paper says to do that
% TODO try RLS formulation in the paper2 and see if it works
% TODO see if there is a bug in saturateSValues() in extractTarget()

% TODO add noise !!!!!

% [int, p] = planningProblem(e);
% 
% f = 1;
% 
% for a=1:e.n_agents
%     plotBernstein(p(:,:,a), int, f); hold on;
% end



