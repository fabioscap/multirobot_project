close all

% Example adjacency matrix for a graph with 5 nodes
A  = [
    0, 1, 0, 0, 1;
    1, 0, 1, 0, 0;
    0, 1, 0, 1, 0;
    0, 0, 1, 0, 1;
    1, 0, 0, 1, 0;
];
L = adj2laplacian(A);

% Call the function to plot the graph
vizFormation(A);


% 1D consensus
x0 = 10*rand(size(A,1),1);
int = [0 10];

% ground truth convergence value
X = mean(x0);

[t,x] = ode45(@(t,x) -L*x, int, x0);
figure()
plot(t,x); hold on;
plot(int, [X, X], "r", "LineWidth",2)
