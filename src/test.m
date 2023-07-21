%clear all; close all; clc; addpath(genpath(pwd));

% the actual target pose (unknown to the agents)
% [ x; y; theta];
p_t = [ 0; 0];


% the number of agents
n_agents = 5;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.1;

e = Environment(2, n_agents, p_t, tau);
e.sim();

% TODO define how to handle
% orientations: do we need them?
% 2D/3D: adapth the observability index in 2D and the whole estimation
% pipeline

% TODO add noise !!!!!

% [int, p] = planningProblem(e);
% 
% f = 1;
% 
% for a=1:length(e.agents)
%     plotBernstein(p(:,:,a), int, f); hold on;
% 
% end



