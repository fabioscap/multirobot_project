clear all; close all; clc; addpath(genpath(pwd));

%  reproducibility
rng(33333)

dim =3;

% the actual target pose (unknown to the agents)
% [ x; y];
p_t = zeros(dim, 1);
if dim ==2
    p_t = [20;-10];
elseif dim ==3
    p_t = [10;5;1];
end

% the number of agents
n_agents = 6;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 1.5;

e = EnvironmentDec(dim, n_agents, p_t, tau);
[T, X] = e.sim();