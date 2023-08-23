clear all; close all; clc; addpath(genpath(pwd));

dim =3;

% the actual target pose (unknown to the agents)
% [ x; y];
p_t = zeros(dim, 1);

p_t = [10;20;5];

% the number of agents
n_agents = 5;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.5;

e = EnvironmentDec(dim, n_agents, p_t, tau);
[T, X] = e.sim();