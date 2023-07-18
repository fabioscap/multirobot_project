clear all; close all; clc; addpath(genpath(pwd));

% the actual target pose (unknown to the agents)
% [ x; y; theta];
p_t = [ 0; 0; 0];


% the number of agents
N = 3;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.1;

e = Environment(2, N, p_t, tau);

% TODO implement de casteljau to split / eval bernstein polynomials
% or better find a matlab library that already has all the implementations

% split is necessary to implement one of the constraints in the optimal
% problem







